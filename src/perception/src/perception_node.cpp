#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "perception/perception.hpp"

int main(int argc, char ** argv)
{
  printf("hello world perception package\n");
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PerceptionNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
