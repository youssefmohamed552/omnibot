#include <cstdio>

#include "localization/ekf_localization.hpp"

int main(int argc, char ** argv)
{
  printf("hello world localization package\n");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EKF_Localization>());
  rclcpp::shutdown();
  return 0;
}
