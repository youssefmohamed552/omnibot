#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "omnibot_msgs/msg/landmark.hpp"
#include "omnibot_msgs/msg/landmarks.hpp"

omnibot_msgs::msg::Landmarks
get_landmarks(){
  omnibot_msgs::msg::Landmarks landmarks;
  landmarks.landmarks.push_back(omnibot_msgs::msg::Landmark());
  landmarks.landmarks.back().position.x = -1.0;
  landmarks.landmarks.back().position.y =  3.0;
  landmarks.landmarks.back().position.z =  0.0;
  landmarks.landmarks.back().id = 0;
  landmarks.landmarks.push_back(omnibot_msgs::msg::Landmark());
  landmarks.landmarks.back().position.x =  2.0;
  landmarks.landmarks.back().position.y =  3.0;
  landmarks.landmarks.back().position.z =  0.0;
  landmarks.landmarks.back().id = 1;
  landmarks.landmarks.push_back(omnibot_msgs::msg::Landmark());
  landmarks.landmarks.back().position.x =  2.0;
  landmarks.landmarks.back().position.y =  1.5;
  landmarks.landmarks.back().position.z =  0.0;
  landmarks.landmarks.back().id = 2;
  return landmarks;
}

class LandmarkPublisher: public rclcpp::Node{
  private:
    rclcpp::Publisher<omnibot_msgs::msg::Landmarks>::SharedPtr m_publisher;

  public:
    LandmarkPublisher()
      : Node("landmark_node")
    {
      m_publisher = this->create_publisher<omnibot_msgs::msg::Landmarks>("landmarks", 10);

      sleep(3);
      std::cout << "publish landmarks" << std::endl;
      m_publisher->publish(get_landmarks());
    }

    virtual ~LandmarkPublisher(){}
};


int main(int argc, char** argv){
  std::cout << "landmarks Node" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LandmarkPublisher>());
  rclcpp::shutdown();
  return 0;
}
