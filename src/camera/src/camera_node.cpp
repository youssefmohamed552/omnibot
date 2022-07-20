#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "camera/image_publisher.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImagePublisher>();

  rclcpp::spin(node);

  

  node->release();
  cv::destroyAllWindows();

  rclcpp::shutdown();


  printf("hello world camera package\n");

  return 0;
}
