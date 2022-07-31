#include <iostream>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"


geometry_msgs::msg::PoseStamped
pose_stamped(const double& x, const double& y){
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.pose.position.x = x;
  pose_stamped.pose.position.y = y;
  return pose_stamped;
}

class PathNode: public rclcpp::Node{
  private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_publisher;

    nav_msgs::msg::Path m_path;
  public:
    PathNode()
      : Node("path_node")
    {
      m_path.poses.push_back(pose_stamped(0.0, 0.0));
      m_path.poses.push_back(pose_stamped(0.5, 0.5));
      m_path.poses.push_back(pose_stamped(1.0, 1.0));
      m_path.poses.push_back(pose_stamped(1.5, 1.5));
      m_path.poses.push_back(pose_stamped(2.0, 2.0));
      m_path.poses.push_back(pose_stamped(2.5, 2.5));
      m_path.poses.push_back(pose_stamped(3.0, 3.0));
      m_path.poses.push_back(pose_stamped(3.5, 3.0));
      m_path.poses.push_back(pose_stamped(4.0, 3.0));
      m_path.poses.push_back(pose_stamped(4.5, 3.0));
      m_path.poses.push_back(pose_stamped(5.0, 3.0));

      m_path_publisher = this->create_publisher<nav_msgs::msg::Path>("path", 10);
      sleep(4);

      std::cout << "pulishing path msg" << std::endl;
      m_path_publisher->publish(m_path);
    }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathNode>());
  rclcpp::shutdown();
  return 0;
}
