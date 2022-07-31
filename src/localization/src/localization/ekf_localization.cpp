#include "localization/ekf_localization.hpp"

double
quaternion_to_yaw(const geometry_msgs::msg::Quaternion& quaternion){
  double w = quaternion.w;
  double x = quaternion.x;
  double y = quaternion.y;
  double z = quaternion.z;
  return atan2(2.0f * (w * z + x * y), w * w + x * x - y * y - z * z); 
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
cap_angle(double& theta){
  while(theta < 0){
    theta += (2.0 * M_PI);
  }
  
  while(theta >= 2 * M_PI){
    theta -= (2.0 * M_PI);
  }
}

bool
out_of_range(const double& w){
  return w > 0.00001 || w < -0.00001;
}

EKF_Localization::
EKF_Localization()
  : Node("ekf_node"),
    m_mu(0.0, 0.0, 0.0),
    m_u(0.0, 0.0),
    m_sigma(Eigen::MatrixXd::Zero(3, 3)),
    m_alpha(Eigen::VectorXd::Zero(6)),
    m_Qt(Eigen::MatrixXd::Zero(3, 3)),
    m_z(),
    m_m()
{
  // m_odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&EKF_Localization::handle_odom, this, std::placeholders::_1));
  m_twist_subscriber = this->create_subscription<geometry_msgs::msg::Twist>("cmd", 10, std::bind(&EKF_Localization::handle_twist, this, std::placeholders::_1));
  m_landmark_subscriber = this->create_subscription<omnibot_msgs::msg::Landmarks>("landmarks", 10, std::bind(&EKF_Localization::handle_landmark, this, std::placeholders::_1));
  m_observation_subscriber = this->create_subscription<omnibot_msgs::msg::Observations>("observations", 10, std::bind(&EKF_Localization::handle_observation, this, std::placeholders::_1));
  m_est_odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>("est_odom", 10);
  auto rate = std::chrono::milliseconds((int)(1000.0 / FREQ));
  m_timer = this->create_wall_timer(rate, std::bind(&EKF_Localization::publish_est_odom, this));

  m_alpha(0) = 0.01;
  m_alpha(1) = 0.01;
  m_alpha(2) = 0.01;
  m_alpha(3) = 0.01;
  m_alpha(4) = 0.01;
  m_alpha(5) = 0.01;

  m_Qt(0,0) = 0.01 * 0.01;
  m_Qt(1,1) = 0.01 * 0.01;
  m_Qt(2,2) = 0.01 * 0.01;

}

EKF_Localization::
~EKF_Localization(){}


// void
// EKF_Localization::
// handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
  // m_mu(0) = msg->pose.pose.position.x;
  // m_mu(1) = msg->pose.pose.position.y;
  // m_mu(2) = quaternion_to_yaw(msg->pose.pose.orientation);
  // return;
// }

void
EKF_Localization::
handle_twist(const geometry_msgs::msg::Twist::SharedPtr msg){
  m_u(0) = msg->linear.x;
  m_u(1) = msg->angular.z;
  return;
}

void
EKF_Localization::
handle_landmark(const omnibot_msgs::msg::Landmarks::SharedPtr msg){
  std::cout << "got landmarks" << std::endl;
  for(const auto& landmark: msg->landmarks){
    m_m[landmark.id] = landmark.position;
  }
  return;
}

void
EKF_Localization::
handle_observation(const omnibot_msgs::msg::Observations::SharedPtr msg){
  std::cout << "got observation size: " << msg->observations.size() << std::endl;
  m_z = *msg;
  return;
}

void
EKF_Localization::
publish_est_odom(){
  step(1.0 / FREQ);
  std::cout << "mu:\n" << m_mu << std::endl;
  nav_msgs::msg::Odometry est_odom;
  est_odom.pose.pose.position.x = m_mu(0);
  est_odom.pose.pose.position.y = m_mu(1);
  est_odom.pose.pose.orientation = yaw_to_quaternion(m_mu(2));
  m_est_odom_publisher->publish(est_odom);
}


void
EKF_Localization::
step(const double& dt){
  if(m_u == Eigen::VectorXd::Zero(2)) return;
  std::cout << "step" << std::endl;
  double theta = m_mu(2);
  double r = 0.0;
  if(!out_of_range(m_u(1))){
    r = m_u(0) * dt;
  }
  else{
    r = (m_u(0)/m_u(1));
  }
  Eigen::MatrixXd Gt = Eigen::MatrixXd::Zero(3, 3);

  double dx_dtheta = 0.0;
  double dy_dtheta = 0.0;
  if(!out_of_range(m_u(1))){
    dx_dtheta = -r * sin(theta);
    dy_dtheta = r * cos(theta);
  }
  else{
    dx_dtheta = -(r * cos(theta)) + (r * cos(theta + (m_u(1) * dt)));
    dy_dtheta = -(r * sin(theta)) + (r * sin(theta + (m_u(1) * dt)));
  }
  Gt(0, 0) = 1;
  Gt(1, 1) = 1;
  Gt(0, 2) = dx_dtheta;
  Gt(1, 2) = dy_dtheta;
  Gt(2, 2) = 1;

  Eigen::MatrixXd Vt = Eigen::MatrixXd::Zero(3, 2);

  double dx_dv = 0.0;
  double dy_dv = 0.0;
  double dx_dw = 0.0;
  double dy_dw = 0.0;

  if(!out_of_range(m_u(1))){
    dx_dv = cos(theta) * dt;
    dy_dv = sin(theta) * dt;
  }
  else{
    dx_dv = (-(sin(theta)) + (sin(theta + (m_u(1) * dt)))) / m_u(1);
    dy_dv = (cos(theta) - cos(theta + (m_u(1) * dt))) / m_u(1);
    dx_dw = ((m_u(0) * (sin(theta) - sin(theta + (m_u(1) * dt)))) / (m_u(1) * m_u(1))) + ((m_u(0) * (cos(theta + (m_u(1) * dt))) * dt) / m_u(1));
    dy_dw = -((m_u(0) * (cos(theta) - cos(theta + (m_u(1) * dt)))) / (m_u(1) * m_u(1))) + ((m_u(0) * (sin(theta + (m_u(1) * dt))) * dt) / m_u(1));
  }
  Vt(0, 0) = dx_dv;
  Vt(1, 0) = dy_dv;
  Vt(0, 1) = dx_dw;
  Vt(1, 1) = dy_dw;
  Vt(2, 1) = dt;

  Eigen::MatrixXd Mt = Eigen::MatrixXd::Zero(2, 2);
  Mt(0, 0) = (m_alpha(0) * (m_u(0) * m_u(0))) + (m_alpha(1) * (m_u(1) * m_u(1)));
  Mt(1, 1) = (m_alpha(2) * (m_u(0) * m_u(0))) + (m_alpha(3) * (m_u(1) * m_u(1)));

  Eigen::VectorXd dmu = Eigen::VectorXd::Zero(3);
  if(!out_of_range(m_u(1))){
    dmu(0) = r * cos(theta);
    dmu(1) = r * sin(theta);
  }
  else{
    dmu(0) = -(r * sin(theta)) + (r * sin(theta + (m_u(1) * dt)));
    dmu(1) = (r * cos(theta)) - (r * cos(theta + (m_u(1) * dt)));
  }
  dmu(2) = m_u(1) * dt;

  Eigen::VectorXd mu_t = m_mu + dmu;
  cap_angle(mu_t(2));

  Eigen::MatrixXd sigma_t = (Gt * m_sigma * Gt.transpose()) + (Vt * Mt * Vt.transpose());

  for(const auto& obs: m_z.observations){
    Eigen::VectorXd c = Eigen::VectorXd::Zero(3);
    c(0) = obs.range;
    c(1) = obs.bearing;
    c(2) = obs.signature;
    int j = obs.signature;

    double dx = m_m[j].x - mu_t(0);
    double dy = m_m[j].y - mu_t(1);
    double q = (dx * dx) + (dy * dy);

    Eigen::VectorXd z_t = Eigen::VectorXd::Zero(3);
    z_t(0) = sqrt(q);
    z_t(1) = atan2(m_m[j].y - mu_t(1), m_m[j].x - mu_t(0)) - mu_t(2);
    z_t(2) = j;

    cap_angle(z_t(1));

    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 3);
    H(0, 0) = -(dx / sqrt(q));
    H(0, 1) = -(dy / sqrt(q));
    H(1, 0) = dy / q;
    H(1, 1) = -dx / q;
    H(1, 2) = -1;

    Eigen::MatrixXd St = (H * sigma_t * H.transpose()) + m_Qt;
    Eigen::MatrixXd Kt = sigma_t * H.transpose() * St.inverse();
    mu_t = mu_t + (Kt * (c - z_t));
    cap_angle(mu_t(2));
    sigma_t = (Eigen::MatrixXd::Identity(3, 3) - (Kt * H)) * sigma_t;
  }
  m_mu = mu_t;
  m_sigma = sigma_t;

  m_z.observations.clear();

}
