#ifndef IMAGE_PUBLISHER_HPP
#define IMAGE_PUBLISHER_HPP

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>

class ImagePublisher: public rclcpp::Node
{
  private:
    image_transport::CameraPublisher m_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;
    cv::VideoCapture m_cap;
    cv::Mat m_image;
    sensor_msgs::msg::CameraInfo m_camera_info;
    std::string m_window_name = "Omnibot View";

    const int deviceID = 0;
    const int apiID = cv::CAP_ANY;

    double m_publish_rate;


  public:
    ImagePublisher();
    virtual ~ImagePublisher();
    void timer_callback();
    void release();
};

#endif /* IMAGE_PUBLISHER_HPP */
