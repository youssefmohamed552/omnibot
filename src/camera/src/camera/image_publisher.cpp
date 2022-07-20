#include "camera/image_publisher.hpp"


ImagePublisher::
ImagePublisher()
: Node("camera_node")
{
  m_publish_rate = this->declare_parameter("publish_rate", static_cast<double>(30));
  m_publisher = image_transport::create_camera_publisher(this, "image_raw");

  m_timer = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / m_publish_rate)), std::bind(&ImagePublisher::timer_callback, this));
  m_cap.open(0);
  CV_Assert(m_cap.isOpened());
}

ImagePublisher::
~ImagePublisher(){}

void
ImagePublisher::
timer_callback()
{
  if(!m_cap.isOpened()) {
    std::cerr << "ERROR! unable to open camera\n";
  }

  // get image from sensor
  m_cap >> m_image;
  if (m_image.empty())
    return;

  // form image to an sensor_msgs/msg/image
  sensor_msgs::msg::Image::SharedPtr out_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", m_image).toImageMsg();
  out_img->header.stamp = rclcpp::Clock().now();
  m_camera_info.header.stamp = out_img->header.stamp;

  // publish image
  m_publisher.publish(*out_img, m_camera_info);
}

void
ImagePublisher::
release(){
  m_cap.release();
}


