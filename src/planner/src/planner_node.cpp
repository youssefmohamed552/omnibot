#include <cstdio>
#include "rclcpp/rclcpp.hpp"

#include "planner/rrt_star.hpp"

int main(int argc, char ** argv)
{
  printf("hello world planner package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRTStar>());
  rclcpp::shutdown();
  return 0;
}
