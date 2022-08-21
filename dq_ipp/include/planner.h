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
#include "path_generator.h"

using namespace std;
using namespace std::chrono; 
using namespace Eigen;

class voxblox_class;
class ray_caster_class;
class frontier_class;
class path_generator_class;

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
    std::unique_ptr<path_generator_class> pg_;

    // variables
    Eigen::Vector3d g_current_position; // global
    Eigen::Quaterniond g_current_orientation;

    std::vector<SubGoal> best_goal;
    int best_idx;

    nav_msgs::Path rrt_result;
    deque<SubGoal> waypoints;
    deque<SubGoal>::iterator iter;

    // params  
    // constants
    double c_voxel_size;

    double begin;
    // flag
    bool f_planning;    // first flag
    bool f_control;     // controller flag
    bool f_keep_rrt;
    bool f_isDirect;

    // Visualization
    virtual void v_voxels(std::vector<Eigen::Vector3d> voxels) = 0;
    virtual void v_frontiers(bool isSurface) = 0;
    virtual void v_rrt_path() = 0;

};


#endif