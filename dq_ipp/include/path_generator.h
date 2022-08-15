#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include "utility.hpp"

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 

#include "voxblox.h"

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

class path_generator_class{
public:
    explicit path_generator_class(voxblox_class& map_, const ros::NodeHandle& nh);
    virtual ~path_generator_class() = default;

    void get_param(const ros::NodeHandle& nh);

    void ex_func_uav_pose(const Eigen::Vector3d& cur_pos, 
                            const Eigen::Quaterniond& cur_ori);
   

private:
    voxblox_class& map_;    // voxblox map object
    // if you need ray_caster or frontier object then
    // you have to 
    // 1. include "frontier.h"
    // 2. add object variable 'frontier_class& ftr_';
    // 3. add class_generator parameter. 
    // e.g. path_generator_class(... , frontier_class& ftr_);

    // params
    double p_parameter_name;    // sample
    
    // constants
    double c_voxel_size;    // voxel size

    // variables
    Eigen::Vector3d uav_cur_pos;    // sample variable
    Eigen::Quaterniond uav_cur_ori;

};

#endif