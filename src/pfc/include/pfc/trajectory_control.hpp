#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "omnibot_msgs/msg/trajectories.hpp"
#include "omnibot_msgs/msg/trajectory.hpp"

#define FREQ 10.0
#define MAX_LEN 1.0
#define MAX_ANGLE (M_PI/16.0)
#define NUM_POINTS 10
#define NUM_TRAJ 20

class TrajectoryControl: public rclcpp::Node{
  private:
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr m_path_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscription;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_twist_publisher;
    rclcpp::Publisher<omnibot_msgs::msg::Trajectories>::SharedPtr m_trajectories_publisher;
    rclcpp::Publisher<omnibot_msgs::msg::Trajectory>::SharedPtr m_trajectory_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_point_publisher;

    rclcpp::TimerBase::SharedPtr m_timer;

    nav_msgs::msg::Path m_path;
    nav_msgs::msg::Odometry m_odom;
    geometry_msgs::msg::Twist m_twist;
    omnibot_msgs::msg::Trajectories m_trajectories;
    omnibot_msgs::msg::Trajectory m_trajectory;
    unsigned int m_pose_index;
    
  public:
    TrajectoryControl();
    virtual ~TrajectoryControl();
    void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void handle_path(const nav_msgs::msg::Path::SharedPtr msg);
    void timer_callback();
    void step(const double& dt);
};
