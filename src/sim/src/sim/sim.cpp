#include <iostream>

#include "sim/sim.hpp"

double
random_number(const double& b){
  return (-b + 2.0 * b * (double) (rand() % 1000) / 1000.0);
}

double
sample_normal_distribution(double b_squared){
  double total = 0.0;
  double b = sqrt(b_squared);
  for(unsigned int i = 0; i < 12; i++)
    total += random_number(b);
  return 0.5 * total;
}

geometry_msgs::msg::Quaternion
yaw_to_quaternion(const double& yaw){
  geometry_msgs::msg::Quaternion q;
  q.w = cos(yaw / 2.0);
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(yaw / 2.0);
  return q;
}

void
x_to_odom(const Eigen::Vector3d& x, nav_msgs::msg::Odometry& odom){
  odom.pose.pose.position.x = x(0);
  odom.pose.pose.position.y = x(1);
  odom.pose.pose.orientation = yaw_to_quaternion(x(2));
}

void
cap_angle(double& theta){
  while(theta < 0){
    theta += (2 * M_PI);
  }

  while(theta >= (2 * M_PI)){
    theta -= (2 * M_PI);
  }
}


Sim::
Sim()
  : Node("sim_node"),
    m_x(0.0, 0.0, 0.0),
    m_u(0.0, 0.0, 0.0),
    m_t(0.0),
    m_alpha1(0.005),
    m_alpha2(0.05),
    m_alpha3(0.005),
    m_alpha4(0.05),
    m_alpha5(0.005),
    m_alpha6(0.05)
{
  m_noise_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  m_observations_publisher = this->create_publisher<omnibot_msgs::msg::Observations>("observations", 10);
  m_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("perfect_odom", 10);
  m_twist_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd", 10, std::bind(&Sim::handle_twist, this, std::placeholders::_1));
  m_landmark_subscriber = this->create_subscription<omnibot_msgs::msg::Landmarks>("landmarks", 10, std::bind(&Sim::handle_landmarks, this, std::placeholders::_1));
  auto rate = std::chrono::milliseconds(((int)(1000.0/FREQ)));
  m_timer = this->create_wall_timer(rate , std::bind(&Sim::timer_callback, this));
}

Sim::
~Sim(){}

void
Sim::
timer_callback(){
  step(1.0/FREQ);

  nav_msgs::msg::Odometry m_odom;
  nav_msgs::msg::Odometry m_noise_odom;

  x_to_odom(m_x, m_odom);
  x_to_odom(m_x_hat, m_noise_odom);

  omnibot_msgs::msg::Observations observations = compute_observations();

  m_odom_publisher->publish(m_odom);
  m_noise_odom_publisher->publish(m_noise_odom);
  m_observations_publisher->publish(observations);
  return;
}

void
Sim::
step(const double& dt){
  if(m_u == Eigen::VectorXd::Zero(3)) return;

  // create noise model
  Eigen::Vector3d u_hat(m_u);
  Eigen::Vector3d sigma(0.0, 0.0, 0.0);
  sigma(0) = sample_normal_distribution((m_alpha1 * m_u(0) * m_u(0)) + (m_alpha2 * m_u(1) * m_u(1)));
  sigma(1) = sample_normal_distribution((m_alpha3 * m_u(0) * m_u(0)) + (m_alpha4 * m_u(1) * m_u(1)));
  u_hat = m_u + sigma;


  // compute model with noise
  Eigen::Vector3d dx_hat(0.0, 0.0, 0.0);
  double v_w_hat = u_hat(0) / u_hat(1);
  double d_theta_hat = u_hat(1) * dt;
  dx_hat(0) = - (v_w_hat * sin(m_x_hat(2))) + (v_w_hat * sin(m_x_hat(2) + d_theta_hat));
  dx_hat(1) = (v_w_hat * cos(m_x_hat(2))) - (v_w_hat * cos(m_x_hat(2) + d_theta_hat));
  dx_hat(2) = d_theta_hat;
  m_x_hat += dx_hat;
  cap_angle(m_x_hat(2));

  // compute model without noise
  Eigen::Vector3d dx(0.0, 0.0, 0.0);
  double v_w = m_u(0) / m_u(1);
  double d_theta = m_u(1) * dt;
  dx(0) = - (v_w * sin(m_x(2))) + (v_w * sin(m_x(2) + d_theta));
  dx(1) = (v_w * cos(m_x(2))) - (v_w * cos(m_x(2) + d_theta));
  dx(2) = d_theta;
  m_x += dx;
  cap_angle(m_x(2));

  m_t += dt;

  // print updates
  std::cout << "u_hat" << std::endl;
  std::cout << u_hat << std::endl;
  std::cout << "u" << std::endl;
  std::cout << m_u << std::endl;
  std::cout << "x_hat" << std::endl;
  std::cout << m_x_hat << std::endl;
  std::cout << "x" << std::endl;
  std::cout << m_x << std::endl;
}


void
Sim::
handle_twist(const geometry_msgs::msg::Twist::SharedPtr msg){
  m_u(0) = msg->linear.x;
  m_u(1) = msg->angular.z;
  return;
}

void
Sim::
handle_landmarks(const omnibot_msgs::msg::Landmarks::SharedPtr msg){
  std::cout << "got landmarks" << std::endl;
  m_landmarks = *msg;
  return;
}

omnibot_msgs::msg::Observation
get_observation(const Eigen::Vector3d& x, const omnibot_msgs::msg::Landmark& l){
  omnibot_msgs::msg::Observation o;
  const double dy = l.position.y - x(1);
  const double dx = l.position.x - x(0);
  o.range = sqrt((dy * dy) + (dx * dx));
  o.bearing = atan2(dy, dx) - x(2);
  cap_angle(o.bearing);
  o.signature = l.id;
  return o;
}


omnibot_msgs::msg::Observations
Sim::
compute_observations(){
  omnibot_msgs::msg::Observations observations;
  for(const auto& landmark: m_landmarks.landmarks){
    observations.observations.push_back(get_observation(m_x, landmark));
  }
  return observations;
}

