#include "planner.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

planner_class::planner_class(const ros::NodeHandle& nh) 
        : f_planning(false)  {
    map_.reset(new voxblox_class(nh));
    ray_.reset(new ray_caster_class(*map_, nh));
    ft_.reset(new frontier_class(*map_, *ray_, nh));
    pg_.reset(new path_generator_class(*map_, nh));
    get_param(nh);
}

void planner_class::get_param(const ros::NodeHandle& nh)    {
    nh.param("/planner_node/p_verbose", p_verbose, true);
    c_voxel_size = map_->getVoxelSize();
    f_control = false;
}

void planner_class::test()  {
    std::cout << " ---- " << std::endl;
    new_voxels.clear();
    std::cout << "====== " << std::endl;
    // Raycasting : get visible voxels
    tic();
    ray_->getVisibleVoxels(&new_voxels, g_current_position, g_current_orientation);
    new_voxels.erase(std::unique(new_voxels.begin(), new_voxels.end()), new_voxels.end());
    
    // Generate frontiers
    ft_->searchFrontiers(new_voxels);
    // Fill frontier information e.g. id, viewpoints
    ft_->computeFrontiersToVisit();
    // Get best next viewpoint, output = best_goal
    ft_->getTopViewpointsInfo(g_current_position, g_current_orientation, best_goal);
    Eigen::Quaterniond q;
    ft_->yaw2orientation(best_goal[0].g_yaw, q);
    //////////////////////////////////////////////////
    toc();
    if (!f_control) {
        tic();
        rrt_result = pg_->ex_func_uav_pose(g_current_position, g_current_orientation, 
                                            best_goal[0].g_pos, q);
        toc();
        if (rrt_result.poses.size() > 0)    {
            tic();
            waypoints.clear();
            for (int i = 0; i < rrt_result.poses.size(); ++i)   {
                SubGoal tmp_;
                tmp_.g_pos[0] = rrt_result.poses[i].pose.position.x;
                tmp_.g_pos[1] = rrt_result.poses[i].pose.position.y;
                tmp_.g_pos[2] = rrt_result.poses[i].pose.position.z;
                Eigen::Quaterniond q(rrt_result.poses[i].pose.orientation.w, 
                                    rrt_result.poses[i].pose.orientation.x,
                                    rrt_result.poses[i].pose.orientation.y, 
                                    rrt_result.poses[i].pose.orientation.z);
                tmp_.g_yaw = q.toRotationMatrix().eulerAngles(0,1,2)(2);
                tmp_.distance = (tmp_.g_pos - g_current_position).norm();
                waypoints.push_back(tmp_);
            }
            f_control = true;
            toc();
        }    
    }
}