#include <vector>
#include <fstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

double
quaternion_to_yaw(const geometry_msgs::msg::Quaternion& quaternion){
  double x = quaternion.x;
  double y = quaternion.y;
  double z = quaternion.z;
  double w = quaternion.w;

  return atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

class StatsSubscriber: public rclcpp::Node{
  private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_noise_odom_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_subscriber;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_est_odom_subscriber;

    std::vector<nav_msgs::msg::Odometry> m_noise_odom;
    std::vector<nav_msgs::msg::Odometry> m_odom;
    std::vector<nav_msgs::msg::Odometry> m_est_odom;


  public:
    StatsSubscriber()
      : Node("stats_node")
    {
      m_noise_odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&StatsSubscriber::handle_noise_odom, this, std::placeholders::_1));
      m_odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("perfect_odom", 10, std::bind(&StatsSubscriber::handle_odom, this, std::placeholders::_1));
      m_est_odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("est_odom", 10, std::bind(&StatsSubscriber::handle_est_odom, this, std::placeholders::_1));
    }

    virtual ~StatsSubscriber(){
      int size = std::min(std::min(m_noise_odom.size(), m_odom.size()), m_est_odom.size());
      std::ofstream fptr_x;
      std::ofstream fptr_y;
      std::ofstream fptr_theta;
      fptr_x.open("odom_x.csv");
      fptr_y.open("odom_y.csv");
      fptr_theta.open("odom_theta.csv");
      fptr_x << "id,odom,noise,estimated\n";
      fptr_y << "id,odom,noise,estimated\n";
      fptr_theta << "id,odom,noise,estimated\n";
      for(int i = 0; i < size; i++){
        fptr_x << i << "," << m_odom[i].pose.pose.position.x << "," << m_noise_odom[i].pose.pose.position.x << "," << m_est_odom[i].pose.pose.position.x << "\n";
        fptr_y << i << "," << m_odom[i].pose.pose.position.y << "," << m_noise_odom[i].pose.pose.position.y << "," << m_est_odom[i].pose.pose.position.y << "\n";
        fptr_theta << i << "," << quaternion_to_yaw(m_odom[i].pose.pose.orientation) << "," << quaternion_to_yaw(m_noise_odom[i].pose.pose.orientation) << "," << quaternion_to_yaw(m_est_odom[i].pose.pose.orientation) << "\n";
      }
    }
    void handle_noise_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
      m_noise_odom.push_back(*msg);
      return;
    }
    void handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
      m_odom.push_back(*msg);
      return;
    }
    void handle_est_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
      m_est_odom.push_back(*msg);
      return;
    }
};

int main(int argc, char ** argv){
  std::cout << "getting stats" << std::endl;
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatsSubscriber>());
  rclcpp::shutdown();
  return 0;
}
