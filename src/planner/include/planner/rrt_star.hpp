#ifndef RRT_STAR_HPP
#define RRT_STAR_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "omnibot_msgs/msg/rrt.hpp"

#define FREQ 0.1
#define DIST_THRESHOLD 0.3
#define NEIGHBORING_DIST 1.5
#define DIST_TO_GOAL 2.0

class RRTNode{
  private:
    RRTNode* m_parent;
    geometry_msgs::msg::Pose m_pose;
    std::vector<RRTNode*> m_children;
    double m_cost;
  public:
    RRTNode(const geometry_msgs::msg::Pose& pose, RRTNode* parent, const double& cost);
    RRTNode(const RRTNode& other);
    virtual ~RRTNode();
    RRTNode* add_child(const geometry_msgs::msg::Pose& pose);
    bool is_goal_close(const geometry_msgs::msg::Pose& pose);
    RRTNode* parent() const;
    std::vector<RRTNode*> children() const;
    geometry_msgs::msg::Pose pose() const;
    double cost() const;
    bool is_leaf() const;
};

class RRTStar: public rclcpp::Node {
  private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_publisher;
    rclcpp::Publisher<omnibot_msgs::msg::RRT>::SharedPtr m_rrt_publisher;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr m_random_point_publisher;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr m_goal_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscriber;

    rclcpp::TimerBase::SharedPtr m_timer;

    nav_msgs::msg::Path m_path;
    nav_msgs::msg::Odometry m_odom;
    geometry_msgs::msg::Pose m_goal;
    omnibot_msgs::msg::RRT m_rrt;
    bool is_goal_recieved;
    
  public:
    RRTStar();
    virtual ~RRTStar();
    void handle_goal(const geometry_msgs::msg::Pose::SharedPtr msg);
    void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
    void add_node(RRTNode* node, const geometry_msgs::msg::Pose& pose);
    void solve();
};

#endif /* RRT_STAR_HPP */
