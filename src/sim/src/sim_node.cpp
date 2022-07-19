#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sim/sim.hpp"

int main(int argc, char ** argv)
{
  printf("hello world sim package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sim>());
  rclcpp::shutdown();
  return 0;
}
