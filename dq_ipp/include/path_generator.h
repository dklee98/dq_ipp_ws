#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include "utility.hpp"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 
#include <vector>

#include "voxblox.h"
#include "rrt_star.h"

using namespace std;

class path_generator_class{
public:
    explicit path_generator_class(voxblox_class& map_, const ros::NodeHandle& nh);
    virtual ~path_generator_class() = default;

    void get_param(const ros::NodeHandle& nh);

    nav_msgs::Path ex_func_uav_pose(const Eigen::Vector3d& cur_pos, 
                                    const Eigen::Quaterniond& cur_ori,
                                    const Eigen::Vector3d& goal_pos,
                                    const Eigen::Quaterniond& goal_ori);

    bool checkDirectPath(const Eigen::Vector3d& cur_pos, 
                        const Eigen::Quaterniond& cur_ori,
                        const Eigen::Vector3d& goal_pos,
                        const Eigen::Quaterniond& goal_ori);

    bool checkPoint(const Eigen::Vector3d& pos);

private:
    // voxblox map object
    voxblox_class& map_;

    // RRT* ptr
    shared_ptr<RRT_STAR> rrt_star_ = nullptr;

    // params
    int p_max_rrt_iteration;
    double p_rrt_collision_r;
    double p_extension_range;
    double p_goal_reached_tolerance_distance;
    vector<double> p_map_min, p_map_max;
};

#endif