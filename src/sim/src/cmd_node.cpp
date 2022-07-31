#include <iostream>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


class CommandPublisher: public rclcpp::Node{
  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_publisher;
  public:
    CommandPublisher()
      : Node("cmd_publisher_node")
    {
      m_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd", 10);

      geometry_msgs::msg::Twist m_twist_msg;
      m_twist_msg.linear.x = 0.5;
      m_twist_msg.angular.z = 0.5;

      sleep(5);

      std::cout << "publishing twist msg" << std::endl;
      m_publisher->publish(m_twist_msg);
    }
};

int main(int argc, char ** argv){
  std::cout << "share command" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandPublisher>());
  rclcpp::shutdown();
  return 0;
}
