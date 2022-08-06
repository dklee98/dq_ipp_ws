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
  void expandFrontier(const Eigen::Vector3i& first, Eigen::Vector3d& bbmin, Eigen::Vector3d& bbmax);
  void getSurfaceFrontier();
  void splitLargeFrontiers(list<Frontier>& ftrs, bool isSurface);
  bool splitXY(const Frontier& frontier, list<Frontier>& splits);
  bool splitYZ(const Frontier& frontier, list<Frontier>& splits);
  bool isFrontierChanged(const Frontier& ft);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  
  void computeFrontiersToVisit();
  void sampleViewpoints(Frontier& frontier);

  void yaw2orientation(double yaw, Eigen::Quaterniond& orientation);
  //////////////
  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  int toAddress(const Eigen::Vector3i& id);
  bool knownFree(const Eigen::Vector3i& idx);
  bool knownOccupied(const Eigen::Vector3i& idx);
  bool knownUnknown(const Eigen::Vector3i& idx);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel, int th);
  bool isNeighborOccupied(const Eigen::Vector3i& voxel);
  bool haveOverlap(const Eigen::Vector3d& min1, const Eigen::Vector3d& max1, 
                  const Eigen::Vector3d& min2, const Eigen::Vector3d& max2);
  std::vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  std::vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);

  //////////////
  bool isSpatialFrontiers(const Eigen::Vector3i& id);
  bool isSurfaceFrontiers(const Eigen::Vector3i& id);
  void getVisibleBox(Eigen::Vector3d& bmin, Eigen::Vector3d& bmax,
                       std::vector<Eigen::Vector3d> new_voxels);

  // frontiers
  std::list<Frontier> tmp_spatial_frontiers, tmp_surface_frontiers;
  std::list<Frontier> spatial_frontiers, surface_frontiers;

private:
  voxblox_class& map_;
  ray_caster_class& ray_;
  
  
  // params
  int p_spatial_cluster_min, p_surface_cluster_min;
  int p_down_sample;
  double p_cluster_size_xy;
  double p_vp_rmax, p_vp_rmin, p_vp_dphi, p_vp_min_dist, p_vp_clearance;
  int p_vp_rnum, p_vp_min_visible_num;

  // constants
  double c_voxel_size, c_voxel_size_inv;
  Eigen::Vector3d c_map_origin, c_map_size;
  Eigen::Vector3i c_map_voxel_num;
  
  // Data
  std::vector<char> frontier_flag;
  std::vector<int> removed_ids;
  std::list<Frontier>::iterator first_spatial_ftr, first_surface_ftr;
};





#endif