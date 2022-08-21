#ifndef _RRT_STAR_H_
#define _RRT_STAR_H_

////// KD tree
#include "kdtree.h"
#include "voxblox.h"

////// common headers
#include <random>
#include <vector>
#include <utility>

////// Eigen
#include <Eigen/Eigen>

////// ROS
#include <ros/ros.h>
#include <nav_msgs/Path.h>

using namespace std;
using namespace Eigen;


///////////////////////////////////////////
struct RRT_NODE
{
  Vector3d pos;
  RRT_NODE *parent;
  vector<RRT_NODE *> children;
  ~RRT_NODE()
  {
    for (typename vector<RRT_NODE *>::iterator node_it = children.begin(); node_it != children.end(); ++node_it) 
    {
      delete (*node_it);
      (*node_it) = NULL;
    }
  }

  double cost()
  {
    if (parent)
      return (pos - parent->pos).norm() + parent->cost();
    return 0;
  }
};


///////////////////////////////////////////
class RRT_STAR
{
public:
  RRT_STAR(int max_nodes, double collision_r, double extension_range, vector<double> boundary_min, vector<double> boundary_max);

  nav_msgs::Path execute(shared_ptr<voxblox::EsdfServer> esdf_ptr, const Vector3d &goal, 
                        const Vector3d &start, const Eigen::Quaterniond& goal_ori, 
                        const Eigen::Quaterniond& start_ori, const double &goal_tolerance_dist=1.0);

  bool collisionLine(shared_ptr<voxblox::EsdfServer> esdf_ptr, const Vector3d &p1, const Vector3d &p2, const double &r);
  bool collisionPoint(shared_ptr<voxblox::EsdfServer> esdf_ptr, const Vector3d &pt, const double &r);

  Vector3d sample(const Vector3d &start);
  RRT_NODE *chooseParent(kdtree *kd_tree, const Vector3d &node, const double &l);
  void rewire(shared_ptr<voxblox::EsdfServer> esdf_ptr, kdtree *kd_tree, RRT_NODE *new_node, const double &l, const double &r);
  Vector3d getNewPos(const Vector3d &sampled, const Vector3d &parent, const double &l);
  RRT_NODE *addNodeToTree(kdtree *kd_tree, RRT_NODE *parent, const Vector3d &new_pos);
  RRT_NODE *getGoal(shared_ptr<voxblox::EsdfServer> esdf_ptr, kdtree *goal_tree, RRT_NODE *new_node, const double &goal_tolerance_dist, const double &r);
  nav_msgs::Path getBestPath(vector<RRT_NODE *> goals, const Eigen::Vector3d& init_pos, const Eigen::Quaterniond& init_ori,
                            const Eigen::Vector3d& end_pos, const Eigen::Quaterniond& end_ori);

private:
  int max_iterations_;
  double bounding_radius_;
  double extension_range_;
  vector<double> boundary_min_;
  vector<double> boundary_max_;

  std::random_device rd;
  std::mt19937 eng;
  std::uniform_real_distribution<float> distr;
};



RRT_STAR::RRT_STAR(int max_iteration, double collision_r, double extension_range, vector<double> boundary_min, vector<double> boundary_max) :
 max_iterations_(max_iteration), bounding_radius_(collision_r), extension_range_(extension_range), 
 boundary_min_(boundary_min), boundary_max_(boundary_max), eng(rd()), distr(0, 1) {}

nav_msgs::Path RRT_STAR::execute(shared_ptr<voxblox::EsdfServer> esdf_ptr, 
                                const Vector3d &goal, const Vector3d &start,
                                const Eigen::Quaterniond& goal_ori, 
                                const Eigen::Quaterniond& start_ori, 
                                const double &goal_tolerance_dist) {
  srand((unsigned int)time(NULL));

  int max_count_ = 0;

  int N = max_iterations_;
  double l = extension_range_;
  double r = bounding_radius_;
  vector<RRT_NODE*> found_goals;

  kdtree* kd_tree = kd_create(3);
  kdtree* goal_tree = kd_create(3);

  //// pushing goal points
  Vector3d* g = new Vector3d(goal);
  kd_insert3(goal_tree, (*g)[0], (*g)[1], (*g)[2], g);
  
  // Initialize root position
  RRT_NODE* root = new RRT_NODE;
  root->pos[0] = start(0);
  root->pos[1] = start(1);
  root->pos[2] = start(2);
  root->parent = NULL;

  kd_insert3(kd_tree, root->pos[0], root->pos[1], root->pos[2], root);

 
  for (int i = 0; i < N; ++i) {
    // Sample new position
    max_count_++;
    if (max_count_ > N)
      break;
    
    Vector3d z_samp;
    do {z_samp = sample(start);}
    while(collisionPoint(esdf_ptr, z_samp, r));

    // Get nearest neighbour
    RRT_NODE* z_parent = chooseParent(kd_tree, z_samp, l);
    if (!z_parent){
      continue;
    }

    // Calculate position for new node
    Vector3d new_pos = getNewPos(z_samp, z_parent->pos, l);

    if (!collisionLine(esdf_ptr, z_parent->pos, new_pos, r)) {
      // Add node to tree
      RRT_NODE* z_new = addNodeToTree(kd_tree, z_parent, new_pos);
      rewire(esdf_ptr, kd_tree, z_new, l, r);

      // Check if goal has been reached
      RRT_NODE* tmp_goal = getGoal(esdf_ptr, goal_tree, z_new, goal_tolerance_dist, r);
      if (tmp_goal)
      {
        found_goals.push_back(tmp_goal);
      }
    }
    else
    {
      --i;
    }
  }

  nav_msgs::Path path_out = getBestPath(found_goals, goal, goal_ori, start, start_ori);

  delete root;
  kd_free(kd_tree);
  kd_free(goal_tree);

  if (path_out.poses.size()==1){
    nav_msgs::Path empty_out;
    empty_out.header.stamp = ros::Time::now();
    empty_out.header.frame_id = "world";
    return empty_out;
  }
  else  {
    return path_out;
  }
}


Vector3d RRT_STAR::sample(const Vector3d &start)
{
  Vector3d x_samp;
  for (int i = 0; i < 3; ++i)
  {
    do
    {
      x_samp[i] = (boundary_max_[i] - boundary_min_[i]) * distr(eng) + boundary_min_[i];
    } while(x_samp[i] < boundary_min_[i] or x_samp[i] > boundary_max_[i]); // sample only one node between min and max
  }

  return x_samp;
}

RRT_NODE* RRT_STAR::chooseParent(kdtree* kd_tree, const Vector3d &node, const double &l)
{
  kdres* nearest = kd_nearest_range3(kd_tree, node[0], node[1], node[2], l);
  if (kd_res_size(nearest) <= 0) {
    nearest = kd_nearest3(kd_tree, node[0], node[1], node[2]);
  }
  if (kd_res_size(nearest) <= 0) {
    kd_res_free(nearest);
    return NULL;
  }

  RRT_NODE* node_nn = (RRT_NODE*)kd_res_item_data(nearest);
  int i = 0;

  RRT_NODE* best_node = node_nn;
  while (!kd_res_end(nearest)) {
    node_nn = (RRT_NODE*)kd_res_item_data(nearest);
    if (best_node and node_nn->cost() < best_node->cost()){
      best_node = node_nn;
    }
    kd_res_next(nearest);
  }

  kd_res_free(nearest);
  return best_node;
}

void RRT_STAR::rewire(shared_ptr<voxblox::EsdfServer> esdf_ptr, kdtree* kd_tree, RRT_NODE* new_node, const double &l, const double &r){
  RRT_NODE* node_nn;
  kdres* nearest = kd_nearest_range3(kd_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2], l + 0.5);
  while (!kd_res_end(nearest)) {
    node_nn = (RRT_NODE*)kd_res_item_data(nearest);
    if (node_nn->cost() > new_node->cost() + (node_nn->pos - new_node->pos).norm()) {
      if (!collisionLine(esdf_ptr, new_node->pos, node_nn->pos, r))
        node_nn->parent = new_node;
    }
    kd_res_next(nearest);
  }
}

Vector3d RRT_STAR::getNewPos(const Vector3d &sampled, const Vector3d &parent, const double &l){
  Vector3d direction = sampled - parent;
  if (direction.norm() > l)
    direction = l * direction.normalized();

  return parent + direction;
}

RRT_NODE* RRT_STAR::addNodeToTree(kdtree* kd_tree, RRT_NODE* parent, const Vector3d &new_pos){
  RRT_NODE* new_node = new RRT_NODE;
  new_node->pos = new_pos;

  new_node->parent = parent;
  parent->children.push_back(new_node);
  kd_insert3(kd_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2], new_node);

  return new_node;
}

RRT_NODE* RRT_STAR::getGoal(shared_ptr<voxblox::EsdfServer> esdf_ptr, kdtree* goal_tree, RRT_NODE* new_node, const double &goal_tolerance_dist, const double &r){
  kdres* nearest_goal = kd_nearest3(goal_tree, new_node->pos[0], new_node->pos[1], new_node->pos[2]);
  if (kd_res_size(nearest_goal) <= 0)
  {
    kd_res_free(nearest_goal);
    return NULL;
  }
  Vector3d* g_nn = (Vector3d*)kd_res_item_data(nearest_goal);
  kd_res_free(nearest_goal);

  if ((*g_nn - new_node->pos).norm() < goal_tolerance_dist)
    if (!collisionLine(esdf_ptr, new_node->pos, *g_nn, r))
      return new_node;

  return NULL;
}


nav_msgs::Path RRT_STAR::getBestPath(vector<RRT_NODE*> goals,
                                    const Eigen::Vector3d& init_pos, 
                                    const Eigen::Quaterniond& init_ori,
                                    const Eigen::Vector3d& end_pos, 
                                    const Eigen::Quaterniond& end_ori){
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "world";
  if (goals.size() == 0)
  {
    return path;
  }

  RRT_NODE* best_node = goals[0];

  for (int i = 0; i < goals.size(); ++i)
    if (best_node->cost() > goals[i]->cost())
      best_node = goals[i];

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "world";
  p.header.stamp = ros::Time::now();
  p.pose.position.x = init_pos[0];
  p.pose.position.y = init_pos[1];
  p.pose.position.z = init_pos[2];
  p.pose.orientation.x = init_ori.x();
  p.pose.orientation.y = init_ori.y();
  p.pose.orientation.z = init_ori.z();
  p.pose.orientation.w = init_ori.w();
  path.poses.push_back(p);

  RRT_NODE* n = best_node;
  if (n->parent){
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = n->pos[0];
    p.pose.position.y = n->pos[1];
    p.pose.position.z = n->pos[2];
    Quaterniond q;
    Vector3d init(1.0, 0.0, 0.0);
    Vector3d dir(n->parent->pos[0] - n->pos[0], n->parent->pos[1] - n->pos[1], 0.0);
    q.setFromTwoVectors(init, dir);
    p.pose.orientation.x = q.x();
    p.pose.orientation.y = q.y();
    p.pose.orientation.z = q.z();
    p.pose.orientation.w = q.w();
    path.poses.push_back(p);
  }
  for (int id = 0; n->parent; ++id)
  {
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "world";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = n->pos[0];
    p.pose.position.y = n->pos[1];
    p.pose.position.z = n->pos[2];
    Quaterniond q;
    Vector3d init(1.0, 0.0, 0.0);
  // Zero out rotation along x and y axis so only yaw is kept
    Vector3d dir(n->parent->pos[0] - n->pos[0], n->parent->pos[1] - n->pos[1], 0.0);
    q.setFromTwoVectors(init, dir);
    // p.pose.orientation.x = q.x();
    // p.pose.orientation.y = q.y();
    // p.pose.orientation.z = q.z();
    // p.pose.orientation.w = q.w();
    p.pose.orientation.x = end_ori.x();
    p.pose.orientation.y = end_ori.y();
    p.pose.orientation.z = end_ori.z();
    p.pose.orientation.w = end_ori.w();

    path.poses.push_back(p);

    n = n->parent;
  }
  
  p.pose.position.x = end_pos[0];
  p.pose.position.y = end_pos[1];
  p.pose.position.z = end_pos[2];
  p.pose.orientation.x = end_ori.x();
  p.pose.orientation.y = end_ori.y();
  p.pose.orientation.z = end_ori.z();
  p.pose.orientation.w = end_ori.w();
  path.poses.push_back(p);

  return path;
}


bool RRT_STAR::collisionLine(shared_ptr<voxblox::EsdfServer> esdf_ptr, const Vector3d &p1, const Vector3d &p2, const double &r){

  Vector3d direction = (p2 - p1).normalized();
  double length = (p2 - p1).norm();
  int step = length / r + 1;

  for (int i = 1; i <= step; ++i)
  {
    Vector3d search_vec = p1 + direction * length * (double)(i / step);

    double distance = 1000.0;
    esdf_ptr->getEsdfMapPtr()->getDistanceAtPosition(search_vec, &distance);
    if (distance < r){ //if any pt within collision_radius, collision
      return true;
    }
  }

  return false;
}



bool RRT_STAR::collisionPoint(shared_ptr<voxblox::EsdfServer> esdf_ptr, const Vector3d &pt, const double &r){

  double distance = 1000.0;
  esdf_ptr->getEsdfMapPtr()->getDistanceAtPosition(pt, &distance);
  if (distance < r){ //if any pt within collision_radius, collision
    return true;
  }
  else return false;
}



#endif