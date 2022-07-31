#include <unistd.h>
#include <list>
#include <time.h>
#include "planner/rrt_star.hpp"
#include "omnibot_msgs/msg/rrt_link.hpp"


double
compute_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2){
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  return sqrt((dx * dx) + (dy * dy));
  
}

RRTNode::
RRTNode(const geometry_msgs::msg::Pose& pose, RRTNode* parent, const double& cost)
  : m_parent(parent), m_pose(pose), m_cost(cost)
{}

RRTNode::
RRTNode(const RRTNode& other){
  m_parent = other.parent();
  m_pose = other.pose();
  m_children = other.children();
  m_cost = other.cost();
}


RRTNode::
~RRTNode(){}


RRTNode*
RRTNode::
add_child(const geometry_msgs::msg::Pose& pose){
  const double cost = m_cost + compute_distance(pose.position, m_pose.position);
  m_children.push_back(new RRTNode(pose, this, cost));
  return m_children.back();
}

bool
RRTNode::
is_leaf() const{
  return m_children.size() == 0;
}

bool
RRTNode::
is_goal_close(const geometry_msgs::msg::Pose& pose){
  double dist = compute_distance(m_pose.position, pose.position);
  return (dist < DIST_THRESHOLD);
}

RRTNode*
RRTNode::
parent() const { return m_parent; }

double
RRTNode::
cost() const { return m_cost; }

geometry_msgs::msg::Pose
RRTNode::
pose() const { return m_pose; }

std::vector<RRTNode*>
RRTNode::
children() const { return m_children; }


RRTStar::
RRTStar()
  : Node("planner_node")
{
  m_path_publisher = this->create_publisher<nav_msgs::msg::Path>("path", 10);
  m_rrt_publisher = this->create_publisher<omnibot_msgs::msg::RRT>("rrt", 10);
  m_random_point_publisher = this->create_publisher<geometry_msgs::msg::Point>("random_point", 10);
  m_goal_subscriber = this->create_subscription<geometry_msgs::msg::Pose>("goal", 10, std::bind(&RRTStar::handle_goal, this, std::placeholders::_1));
  m_odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>("est_odom", 10, std::bind(&RRTStar::handle_odom, this, std::placeholders::_1));

  m_timer = this->create_wall_timer(std::chrono::milliseconds((int) (1000.0 / FREQ)), std::bind(&RRTStar::solve, this));
  is_goal_recieved = false;
  srand(time(0));
}


RRTStar::
~RRTStar() {}


void
RRTStar::
handle_odom(const nav_msgs::msg::Odometry::SharedPtr msg){
  m_odom = *msg;
  return;
}


void
RRTStar::
handle_goal(const geometry_msgs::msg::Pose::SharedPtr msg){
  m_goal = *msg;
  std::cout << "goal revieved to x: "  << m_goal.position.x << " y: " << m_goal.position.y << std::endl;
  is_goal_recieved = true;
  return;
}

void
print_tree(const RRTNode& rrt, int level){
  std::cout << "level: " << level << std::endl;
  std::cout << "(x,y):(" << rrt.pose().position.x << "," << rrt.pose().position.y << ")" << std::endl;
  if(rrt.is_leaf()){
    std::cout << "leaf" << std::endl;
    return;
  }
  for(const auto child: rrt.children()){
    print_tree(*child, level + 1);
  }
}

std::list<RRTNode*>
find_neighboring_nodes(RRTNode* root, const geometry_msgs::msg::Pose& pose){
  double dist = compute_distance(root->pose().position, pose.position);
  std::list<RRTNode*> nl;
  if(dist < NEIGHBORING_DIST){
    nl.push_back(root);
  }
  for(auto child: root->children()){
    std::list<RRTNode*> nls = find_neighboring_nodes(child, pose);
    nl.splice(nl.end(), nls);
  }
  return nl;
}


RRTNode*
find_nearest_node(RRTNode* root, const geometry_msgs::msg::Pose& pose){
  double dist_root = compute_distance(root->pose().position, pose.position);
  RRTNode* nearest_node = root;
  if(root->is_leaf()){
    return nearest_node;
  }
  for(auto child: root->children()){
    RRTNode* node = find_nearest_node(child, pose);
    double dist_to_node = compute_distance(node->pose().position, pose.position);
    if(dist_to_node < dist_root){
      nearest_node = node;
      dist_root = dist_to_node;
    }
  }
  return nearest_node;
}

geometry_msgs::msg::Pose
truncate_pose(RRTNode* node, const geometry_msgs::msg::Pose pose){
  const double dist = compute_distance(node->pose().position, pose.position);

  if(dist < DIST_THRESHOLD){
    return pose;
  }

  double dx = (DIST_THRESHOLD / dist) * (pose.position.x - node->pose().position.x);
  double dy = (DIST_THRESHOLD / dist) * (pose.position.y - node->pose().position.y);

  geometry_msgs::msg::Pose p;
  p.position.x = node->pose().position.x + dx;
  p.position.y = node->pose().position.y + dy;
  return p;
}

void
RRTStar::
add_node(RRTNode* root, const geometry_msgs::msg::Pose& pose){
  std::list<RRTNode*> neighboring_nodes = find_neighboring_nodes(root, pose);
  RRTNode* node = nullptr;
  double min_cost = 10000000.0;

  for(const auto& neighbor: neighboring_nodes){
    double c = neighbor->cost() + compute_distance(neighbor->pose().position, pose.position);
    if(c < min_cost){
      min_cost = c;
      node = neighbor;
    }
  }

  m_rrt.links.push_back(omnibot_msgs::msg::RRTLink());
  m_rrt.links.back().p1 = pose;
  m_rrt.links.back().p2 = node->pose();

  node->add_child(pose);
}

geometry_msgs::msg::PoseStamped
get_pose_stamped(const geometry_msgs::msg::Pose& pose){
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.pose = pose;
  return pose_stamped;
}

void
RRTStar::
solve(){
  if(!is_goal_recieved) return;

  const double dist_to_goal = compute_distance(m_odom.pose.pose.position, m_goal.position);
  if(dist_to_goal < DIST_TO_GOAL) return;

  RRTNode root(m_odom.pose.pose, nullptr, 0.0);

  // keep going to produce RRT
  RRTNode* goal_node = nullptr;
  while(1){
    // get random pose
    geometry_msgs::msg::Pose random_pose;
    if(rand() % 4 != 0){
      random_pose.position.x = (rand() % 2 == 0)? (rand() % 10000): -(rand() % 10000);
      random_pose.position.y = (rand() % 2 == 0)? (rand() % 10000): -(rand() % 10000);
    }
    else{
      random_pose.position.x = m_goal.position.x;
      random_pose.position.y = m_goal.position.y;
    }
    m_random_point_publisher->publish(random_pose.position);

    // find the nearest node
    RRTNode* node = find_nearest_node(&root, random_pose);

    // adjust pose through threshold
    geometry_msgs::msg::Pose truncated_pose = truncate_pose(node, random_pose);

    // find k nearest nodes at distance d
    // pick optimal node to attach based on cost
    add_node(node, truncated_pose);

    // if goal is close enough then connect it and end
    if(node->is_goal_close(m_goal)){
      std::cout << "node is close to goal" << std::endl;
      goal_node = node->add_child(m_goal);
      break;
    }
    m_rrt_publisher->publish(m_rrt);
  }

  // generate path 
  RRTNode* n = goal_node;
  m_path.poses.push_back(get_pose_stamped(n->pose()));
  while(n->parent()){
    n = n->parent();
    m_path.poses.push_back(get_pose_stamped(n->pose()));
  }
  std::reverse(m_path.poses.begin(), m_path.poses.end());

  // publish path
  m_path_publisher->publish(m_path);
  m_path.poses.clear();
  m_rrt.links.clear();
}
