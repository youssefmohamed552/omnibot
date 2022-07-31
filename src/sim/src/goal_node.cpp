#include <iostream>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"



class GoalNode: public rclcpp::Node{
  private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr m_goal_publisher;
    geometry_msgs::msg::Pose m_goal;
  public:
    GoalNode()
      : Node("goal")
    {
      m_goal_publisher = this->create_publisher<geometry_msgs::msg::Pose>("goal", 10);

      m_goal.position.x = 5.0;
      m_goal.position.y = 3.0;


      sleep(3.0);
      m_goal_publisher->publish(m_goal);
    }
};


int
main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalNode>());
  rclcpp::shutdown();
  return 0;
}
