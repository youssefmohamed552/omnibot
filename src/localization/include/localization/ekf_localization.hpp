#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "omnibot_msgs/msg/landmarks.hpp"
#include "omnibot_msgs/msg/observations.hpp"

#include <Eigen/Dense>

#define FREQ 30.0

class EKF_Localization: public rclcpp::Node{
  private:
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr m_twist_subscriber;
    rclcpp::Subscription<omnibot_msgs::msg::Landmarks>::SharedPtr m_landmark_subscriber;
    rclcpp::Subscription<omnibot_msgs::msg::Observations>::SharedPtr m_observation_subscriber;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_est_odom_publisher;
    rclcpp::TimerBase::SharedPtr m_timer;

    Eigen::Vector3d m_mu;
    Eigen::Vector2d m_u;
    // Eigen::Vector3d m_est_mu;
    Eigen::MatrixXd m_sigma;
    Eigen::VectorXd m_alpha;
    Eigen::MatrixXd m_Qt;
    omnibot_msgs::msg::Observations m_z;
    std::map<int, geometry_msgs::msg::Point> m_m;
  public:
    EKF_Localization();
    virtual ~EKF_Localization();
    // void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void handle_twist(const geometry_msgs::msg::Twist::SharedPtr msg);
    void handle_landmark(const omnibot_msgs::msg::Landmarks::SharedPtr msg);
    void handle_observation(const omnibot_msgs::msg::Observations::SharedPtr msg);
    void publish_est_odom();
    void step(const double& dt);

};
