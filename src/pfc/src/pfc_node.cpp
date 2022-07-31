#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "pfc/trajectory_control.hpp"

int main(int argc, char ** argv)
{
  printf("hello world pfc package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryControl>());
  rclcpp::shutdown();
  return 0;
}
