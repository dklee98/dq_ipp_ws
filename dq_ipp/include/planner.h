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
    
    void get_param(const ros::NodeHandle& nh);

    void test();
    bool p_verbose; // for visualization, default true

protected:
    mutex m_mutex;
    std::unique_ptr<voxblox_class> map_;
    std::unique_ptr<ray_caster_class> ray_;
    std::unique_ptr<frontier_class> ft_;

    // variables
    bool f_planning;    // flag
    Eigen::Vector3d g_current_position; // global
    Eigen::Quaterniond g_current_orientation;
    // Eigen::Vector3d target_position;
    // double target_yaw;

    std::vector<Eigen::Vector3d> best_vp_pos;
    std::vector<double> best_vp_yaw;
    Eigen::Vector3d target_pos;
    Eigen::Quaterniond target_ori;

    // params
    // double p_checking_distance; // distance in voxelsizes where we check for known voxels
    // bool p_accurate_frontiers;
    double p_replan_pos_th, p_replan_yaw_th;
    
    // constants
    double c_voxel_size;

    // flag
    bool f_target_reached;

    // Visualization
    virtual void v_voxels(std::vector<Eigen::Vector3d> voxels) = 0;
    virtual void v_frontiers(bool isSurface) = 0;

};


#endif