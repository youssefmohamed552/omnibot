#ifndef PERCEPTION_HPP
#define PERCEPTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include "apriltag/apriltag.h"
#include "omnibot_msgs/msg/observations.hpp"


class PerceptionNode : public rclcpp::Node {
  private:
    double m_tag_size;
    image_transport::Subscriber m_sub;
    rclcpp::Publisher<omnibot_msgs::msg::Observations>::SharedPtr m_obs_pub;

  public:
    PerceptionNode();
    virtual ~PerceptionNode();

    void handleDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    apriltag_detector_t* tag_detector;
    apriltag_family_t* tag_family;
};

#endif /* PERCEPTION_HPP */
