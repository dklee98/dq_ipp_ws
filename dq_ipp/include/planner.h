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

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

class planner_class{
public:
    explicit planner_class();
    virtual ~planner_class() = default;
    
    const static unsigned char SURFACE_F = 0;
    const static unsigned char GENERAL_F = 1;
    
    void get_param();

    void test();

protected:
    voxblox_class *map_ = new voxblox_class();
    ray_caster_class *ray_ = new ray_caster_class(*map_);

    // variables
    bool f_planning;    // flag
    Eigen::Vector3d g_current_position; // global
    Eigen::Quaterniond g_current_orientation;
    Eigen::Vector3d target_position;
    double target_yaw;

    // params
    double p_checking_distance; // distance in voxelsizes where we check for known voxels
    bool p_verbose;
    
    // constants
    double c_voxel_size;
    Eigen::Vector3d c_neighbor_voxels[26];

    bool isFrontierVoxel(const Eigen::Vector3d& voxel);

    // Visualization
    virtual void v_visible_voxels(std::vector<Eigen::Vector3d> voxels, double voxel_size) = 0;
};


#endif