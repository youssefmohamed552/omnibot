#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "perception/perception.hpp"
#include "apriltag/apriltag_pose.h"
#include "apriltag/common/zarray.h"
#include "apriltag/common/image_u8.h"
#include "apriltag/tag36h11.h"
#include "apriltag/common/matd.h"
#include "omnibot_msgs/msg/observation.h"


#define DEBUG 1



PerceptionNode::
PerceptionNode()
: rclcpp::Node("perception_node"), m_tag_size(0.185)
{
  std::cout << "creating Perception Node" << std::endl;
  auto transport = this->declare_parameter("image_transport", "raw");
  // std::string topic = rclcpp::expand_topic_or_service_name( "image", this->get_name(), this->get_namespace());
  image_transport::TransportHints hints(this, transport);
  m_sub = image_transport::create_subscription(this, "image_raw", std::bind(&PerceptionNode::handleDepthImage, this, std::placeholders::_1), hints.getTransport());
  m_obs_pub = this->create_publisher<omnibot_msgs::msg::Observations>("observations", 10);

  tag_family = tag36h11_create();
  tag_detector = apriltag_detector_create();
  apriltag_detector_add_family(tag_detector, tag_family);

}

PerceptionNode::
~PerceptionNode(){
  tag36h11_destroy(tag_family);
}


void
PerceptionNode::
handleDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg){
  std::cout << "got depth image" << std::endl;
  cv_bridge::CvImagePtr cv_image_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

  cv::Mat cv_image_gray(cv_image_ptr->image.rows, cv_image_ptr->image.cols, CV_8UC1);
  cv::cvtColor( cv_image_ptr-> image, cv_image_gray, CV_RGB2GRAY);




  // image_u8_t *image = image_u8_create(cv_image->image.rows, cv_image->image.cols);
  image_u8_t* im = image_u8_create_stride(cv_image_gray.cols,
                                         cv_image_gray.rows,
                                         cv_image_gray.cols);
  im->buf = cv_image_gray.data;
  // copy_image(cv_image, image);

  zarray_t *detections = apriltag_detector_detect(tag_detector, im);

  omnibot_msgs::msg::Observations obs;

  for(int i = 0; i < zarray_size(detections); i++){
    std::cout << "got detections" << std::endl;
    apriltag_detection_t *detection;
    zarray_get(detections, i, &detection);
    std::cout << "tag id: " << detection->id << std::endl;

    apriltag_pose_t pose;
    apriltag_detection_info_t info ={
      .det=detection,
      .tagsize=0.12,
      .fx=1428.03,
      .fy=1420.51,
      .cx=994.5,
      .cy=441.9,
    };

#if DEBUG == 1
    for(int j = 0; j < 4; j++){
      cv::line(cv_image_ptr->image, cv::Point(detection->p[j][0], detection->p[j][1]), cv::Point(detection->p[(j+1)%4][0], detection->p[(j+1)%4][1]), cv::Scalar(0, 255, 0), 2, cv::LINE_8);
    }
    cv::circle(cv_image_ptr->image, cv::Point(detection->c[0], detection->c[1]), 3, cv::Scalar(0, 0, 225), -1);
#endif

    estimate_tag_pose(&info, &pose);
    double t0 = matd_get(pose.t, 0, 0);
    double t1 = matd_get(pose.t, 1, 0);
    std::cout << "t: " << std::endl;
    matd_print(pose.t, "  %lf ");

    double range = sqrt((t0 * t0) + (t1 * t1));
    double bearing = atan2(t1, t0);

    obs.observations.push_back(omnibot_msgs::msg::Observation());
    obs.observations.back().range = range;
    obs.observations.back().bearing = bearing;
    obs.observations.back().signature = detection->id;



    std::cout << "(range,bearing,signature):(" << range << "," << bearing << "," << detection->id << ")" << std::endl;
  }
  m_obs_pub->publish(obs);

  //image_u8_destroy(im);
  //apriltag_detections_destroy(detections);

  cv::imshow("Depth Image", cv_image_ptr->image);
  cv::waitKey(3);
  return;
}
