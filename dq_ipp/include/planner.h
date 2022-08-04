#ifndef PLANNER_H
#define PLANNER_H

#include "utility.hpp"

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 

#include "voxblox.h"
#include "ray_caster.h"
#include "frontier.h"

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

class voxblox_class;
class ray_caster_class;
class frontier_class;

class planner_class{
public:
    explicit planner_class(const ros::NodeHandle& nh);
    virtual ~planner_class() = default;
    
    // const static unsigned char NOT_FRONTIER = 0;
    // const static unsigned char SURFACE_FRONTIER = 1;
    // const static unsigned char SPATIAL_FRONTIER = 2;
    
    void get_param(const ros::NodeHandle& nh);

    void test();

protected:
    // voxblox_class *map_ = new voxblox_class();
    // ray_caster_class *ray_ = new ray_caster_class(*map_);
    std::unique_ptr<voxblox_class> map_;
    std::unique_ptr<ray_caster_class> ray_;
    std::unique_ptr<frontier_class> ft_;

    // variables
    bool f_planning;    // flag
    Eigen::Vector3d g_current_position; // global
    Eigen::Quaterniond g_current_orientation;
    Eigen::Vector3d target_position;
    double target_yaw;

    // uncomment after coding update frontier funciton!!!
    std::vector<Eigen::Vector3d> surface_f;
    std::vector<Eigen::Vector3d> spatial_f;

    // params
    // double p_checking_distance; // distance in voxelsizes where we check for known voxels
    // bool p_accurate_frontiers;
    bool p_verbose;
    
    // constants
    double c_voxel_size;
    // Eigen::Vector3d c_neighbor_voxels[26];
    
    // void get_frontiers(std::vector<Eigen::Vector3d>* surface_result,
    //                             std::vector<Eigen::Vector3d>* spatial_result,
    //                             std::vector<Eigen::Vector3d> voxels);
    // void update_frontiers(std::vector<Eigen::Vector3d>* surface_result,
    //                             std::vector<Eigen::Vector3d>* spatial_result);
    // int isFrontierVoxel(const Eigen::Vector3d& voxel);

    // Visualization
    const static unsigned char V_visible_voxels = 0;
    const static unsigned char V_surface_frontiers = 1;
    const static unsigned char V_spatial_frontiers = 2;
    virtual void v_voxels(std::vector<Eigen::Vector3d> voxels, double voxel_size, int print_type) = 0;
    virtual void v_frontiers() = 0;

};


#endif