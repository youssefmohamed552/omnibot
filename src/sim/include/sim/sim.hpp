#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "omnibot_msgs/msg/landmark.hpp"
#include "omnibot_msgs/msg/landmarks.hpp"
#include "omnibot_msgs/msg/observation.hpp"
#include "omnibot_msgs/msg/observations.hpp"


#define FREQ 30.0

class Sim: public rclcpp::Node{
  private:
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_publisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_noise_odom_publisher;
    rclcpp::Publisher<omnibot_msgs::msg::Observations>::SharedPtr m_observations_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twist_subscriber;

    

    Eigen::Vector3d m_x;
    Eigen::Vector3d m_x_hat;
    Eigen::Vector3d m_u;
    double m_t;
    double m_alpha1;
    double m_alpha2;
    double m_alpha3;
    double m_alpha4;
    double m_alpha5;
    double m_alpha6;
    omnibot_msgs::msg::Landmarks m_landmarks;


  public:
    Sim();
    virtual ~Sim();
    void timer_callback();
    void step(const double& dt);
    void handle_twist(const geometry_msgs::msg::Twist::SharedPtr msg);
    void handle_landmarks(const omnibot_msgs::msg::Landmarks::SharedPtr msg);
    omnibot_msgs::msg::Observations compute_observations();
};
