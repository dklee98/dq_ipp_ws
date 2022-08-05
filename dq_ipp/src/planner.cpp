#include "planner.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

planner_class::planner_class(const ros::NodeHandle& nh) 
        : f_planning(false)  {
    map_.reset(new voxblox_class(nh));
    ray_.reset(new ray_caster_class(*map_, nh));
    ft_.reset(new frontier_class(*map_, *ray_, nh));
    get_param(nh);
}

void planner_class::get_param(const ros::NodeHandle& nh)    {
    nh.param("/p_verbose", p_verbose, true);
    c_voxel_size = map_->getVoxelSize();
}

void planner_class::test()  {
    // test getVisibleVoxels
    std::vector<Eigen::Vector3d> new_voxels;
    // ROS_INFO("getVisiblevoxels");
    // tic();
    ray_->getVisibleVoxels(&new_voxels, g_current_position, g_current_orientation);
    new_voxels.erase(std::unique(new_voxels.begin(), new_voxels.end()), new_voxels.end());
    // toc();
    // ROS_INFO("getFrontiers");
    // tic();
    ft_->searchFrontiers(new_voxels);
    ft_->computeFrontiersToVisit();
    // toc();
    if (p_verbose)  {
        v_voxels(new_voxels);
        v_frontiers();
    }
    
}