#ifndef FRONTIER_H
#define FRONTIER_H

#include "utility.hpp"

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 

#include <pcl/filters/voxel_grid.h>

#include "voxblox.h"
#include "ray_caster.h"

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

class voxblox_class;
class ray_caster_class;

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Eigen::Vector3d pos_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  std::vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  std::vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Eigen::Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  std::vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Eigen::Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  std::list<vector<Vector3d>> paths_;
  std::list<double> costs_;
};


class frontier_class{
public:
  explicit frontier_class(voxblox_class& map_, ray_caster_class& ray_, const ros::NodeHandle& nh);
  virtual ~frontier_class() = default;
  
  void get_param(const ros::NodeHandle& nh);

  void searchFrontiers(std::vector<Eigen::Vector3d> new_voxels);
  void expandFrontier(const Eigen::Vector3i& first);
  void splitLargeFrontiers(list<Frontier>& ftrs);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  bool isFrontierChanged(const Frontier& ft);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);

  void computeFrontiersToVisit();

  //////////////
  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  int toAddress(const Eigen::Vector3i& id);
  bool knownfree(const Eigen::Vector3i& idx);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  std::vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  std::vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);

  //////////////


    
  std::list<Frontier> frontiers, tmp_frontiers;

private:
  voxblox_class& map_;
  ray_caster_class& ray_;
  
  
  // params
  int p_cluster_min;
  int p_down_sample;
  double p_cluster_size_xy, p_cluster_size_z;

  // constants
  double c_voxel_size, c_voxel_size_inv;
  Eigen::Vector3d c_map_origin, c_map_size;
  Eigen::Vector3i c_map_voxel_num;
  
  // Data
  std::vector<char> frontier_flag;
  std::vector<int> removed_ids;
  std::list<Frontier>::iterator first_new_ftr;
};





#endif