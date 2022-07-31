#include "pfc/trajectory_control.hpp"
#include "omnibot_msgs/msg/trajectory_point.hpp"
#include "omnibot_msgs/msg/trajectory.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

geometry_msgs::msg::Quaternion
yaw_to_quaternion(const double& yaw){
  geometry_msgs::msg::Quaternion quaternion;
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = sin(yaw / 2.0);
  quaternion.w = cos(yaw / 2.0);
  return quaternion;
}

double
quaternion_to_yaw(const geometry_msgs::msg::Quaternion& q){
  return atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
}

double 
compute_dist(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2){
  return sqrt(((p1.x - p2.x) * (p1.x - p2.x)) + ((p1.y - p2.y) * (p1.y - p2.y)));
}

bool
out_of_range(const double& w){
  return w > 0.00001 || w < -0.00001;
}

TrajectoryControl::
TrajectoryControl()
  : Node("pfc_node"),
    m_path(),
    m_odom(),
    m_twist(),
    m_trajectories(),
    m_trajectory(),
    m_pose_index(0)
{
  m_path_subscription = this->create_subscription<nav_msgs::msg::Path>("path", 10, std::bind(&TrajectoryControl::handle_path, this, std::placeholders::_1));
  m_odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>("est_odom", 10, std::bind(&TrajectoryControl::handle_odom, this, std::placeholders::_1));
  m_twist_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd", 10);
  m_trajectories_publisher = this->create_publisher<omnibot_msgs::msg::Trajectories>("traj", 10);
  m_trajectory_publisher = this->create_publisher<omnibot_msgs::msg::Trajectory>("main_traj", 10);
  m_point_publisher = this->create_publisher<geometry_msgs::msg::Point>("path_point", 10);

  m_timer = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / FREQ)), std::bind(&TrajectoryControl::timer_callback, this));
}


TrajectoryControl::
~TrajectoryControl(){}


void
TrajectoryControl::
handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
  m_odom = *msg;
  return;
}

void
TrajectoryControl::
handle_path(const nav_msgs::msg::Path::SharedPtr msg){
  m_path = *msg;
  return;
}

void
TrajectoryControl::
timer_callback(){
  step(0.1);
  m_trajectories_publisher->publish(m_trajectories);
  m_trajectory_publisher->publish(m_trajectory);
  m_twist_publisher->publish(m_twist);
}

void
TrajectoryControl::
step(const double& dt){
  if(m_trajectory.points.size() > 0){
    m_twist = m_trajectory.points.back().twist;
  }

  if(m_pose_index >= m_path.poses.size()) return;
  std::cout << "step" << std::endl;
  m_trajectories.trajectories.clear();

  // move point index
  const double dist_threshold = 0.5 * dt * NUM_POINTS;
  while(m_pose_index + 1 < m_path.poses.size() && compute_dist(m_path.poses[m_pose_index].pose.position, m_odom.pose.pose.position) < dist_threshold){
    m_pose_index++;
  }

  if(compute_dist(m_path.poses[m_pose_index].pose.position, m_odom.pose.pose.position) < 0.05){
    m_twist = geometry_msgs::msg::Twist();
    return;
  }

  // generate trajectories
  geometry_msgs::msg::Pose pose = m_odom.pose.pose;
  const double v = 0.5;
  const double delta_angle = (2.0 * MAX_ANGLE) / NUM_TRAJ;
  for(int i = 0; i < NUM_TRAJ; i++){
    omnibot_msgs::msg::Trajectory trajectory;

    const double w = (MAX_ANGLE - (delta_angle * i)) / dt;
    double r = v * dt;
    if(out_of_range(w)){
      r = v / w;
    }

    trajectory.points.push_back(omnibot_msgs::msg::TrajectoryPoint());
    trajectory.points.back().pose = pose;
    trajectory.points.back().twist.linear.x = v;
    trajectory.points.back().twist.angular.z = w;
    double len = 0.0;

    
    //generate points
    for(int j = 1; j < NUM_POINTS; j++){
      omnibot_msgs::msg::TrajectoryPoint trajectory_point;
      const double theta = quaternion_to_yaw(trajectory.points.back().pose.orientation);
      double dx = r * cos(theta);
      double dy = r * sin(theta);
      if(out_of_range(w)){
        dx = -(r * sin(theta)) + (r * sin(theta + (w * dt)));
        dy = (r * cos(theta)) - (r * cos(theta + (w * dt)));
        
      }
      len += sqrt((dx * dx) + (dy * dy));
      std::cout << "len: " << len << std::endl;
      trajectory_point.pose.position.x = trajectory.points.back().pose.position.x + dx;
      trajectory_point.pose.position.y = trajectory.points.back().pose.position.y + dy;
      trajectory_point.pose.orientation = yaw_to_quaternion(theta + (w * dt));
      // std::cout << "after theta: " <<quaternion_to_yaw(trajectory_point.pose.orientation) << std::endl;
      trajectory_point.twist = trajectory.points.back().twist;
      trajectory.points.push_back(trajectory_point);
    }
    trajectory.cost = compute_dist(m_path.poses[m_pose_index].pose.position, trajectory.points.back().pose.position);
    m_trajectories.trajectories.push_back(trajectory);
  }

  double cost = 1000000.0;
  unsigned int chosen_traj_index = 0;

  // find cost
  for(unsigned int i = 0; i < NUM_TRAJ; i++){
    if(m_trajectories.trajectories[i].cost < cost){
      cost = m_trajectories.trajectories[i].cost;
      chosen_traj_index = i;
    }
  }

  // pick the trajectory and twist params
  m_trajectory = m_trajectories.trajectories[chosen_traj_index];
  m_twist = m_trajectory.points[0].twist;
  m_point_publisher->publish(m_path.poses[m_pose_index].pose.position);
}
