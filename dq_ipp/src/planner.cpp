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
    nh.param("/p_replan_pos_th", p_replan_pos_th, 0.35);
    nh.param("/p_replan_yaw_th", p_replan_yaw_th, 0.35);
    c_voxel_size = map_->getVoxelSize();
    f_target_reached = true;
}

void planner_class::test()  {
    // test getVisibleVoxels
    std::vector<Eigen::Vector3d> new_voxels;
    // ROS_INFO("getVisiblevoxels");
    tic();
    ray_->getVisibleVoxels(&new_voxels, g_current_position, g_current_orientation);
    new_voxels.erase(std::unique(new_voxels.begin(), new_voxels.end()), new_voxels.end());
    // toc();
    // ROS_INFO("getFrontiers");
    // tic();
    ft_->searchFrontiers(new_voxels);
    ft_->computeFrontiersToVisit();
    ft_->getTopViewpointsInfo(g_current_position, g_current_orientation, best_vp_pos, best_vp_yaw);
    if (f_target_reached) {
        if (p_verbose)  std::cout << "Target reached!!" << std::endl;
        target_pos = {best_vp_pos[0][0], best_vp_pos[0][1], best_vp_pos[0][2]};
        ft_->yaw2orientation(best_vp_yaw[0], target_ori);
        f_target_reached = false;
    }
    else{
        if ((target_pos - g_current_position).norm() < p_replan_pos_th) {
            double cur_yaw = g_current_orientation.toRotationMatrix().eulerAngles(0,1,2)(2);
            double tar_yaw = target_ori.toRotationMatrix().eulerAngles(0,1,2)(2);
            if (angleDifference(tar_yaw, cur_yaw) < p_replan_yaw_th)    {
                f_target_reached = true;
            }
        }
    }
    toc();
    if (p_verbose)  {
        v_voxels(new_voxels);
        v_frontiers(false); // spatial frontiers
        v_frontiers(true);  // surface frontiers
    }
}