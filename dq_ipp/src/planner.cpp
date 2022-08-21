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
    f_keep_rrt = false;
    f_isDirect = false;
    best_idx = 0;
}

void planner_class::test()  {
    // new_voxels.clear();
    std::vector<Eigen::Vector3d> new_voxels;
    // Raycasting : get visible voxels
    ray_->getVisibleVoxels(&new_voxels, g_current_position, g_current_orientation);
    new_voxels.erase(std::unique(new_voxels.begin(), new_voxels.end()), new_voxels.end());
    
    // Generate frontiers
    ft_->searchFrontiers(new_voxels, g_current_position, g_current_orientation);
    // Fill frontier information e.g. id, viewpoints
    ft_->computeFrontiersToVisit();
    // Get best next viewpoint, output = best_goal
    ft_->getTopViewpointsInfo(g_current_position, g_current_orientation, best_goal);
    Eigen::Quaterniond q;
    ft_->yaw2orientation(best_goal[best_idx].g_yaw, q);
    //////////////////////////////////////////////////
    if (!f_control) {
        // if (!pg_->checkDirectPath(g_current_position, g_current_orientation, 
        //                                     best_goal[best_idx].g_pos, q)) {
        //     std::cout << "direct path" << std::endl;
        //     waypoints.clear();
        //     waypoints.push_back(best_goal[best_idx]);
        //     f_isDirect = true;
        //     f_control = true;
        // }
        // else{
            rrt_result = pg_->ex_func_uav_pose(g_current_position, g_current_orientation, 
                                                best_goal[best_idx].g_pos, q);
            if (rrt_result.poses.size() > 0)    {
                f_keep_rrt = false;
                best_idx = 0;
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
            }
            else{
                if (!f_keep_rrt)   {
                    std::cout << "no rrt, timer begin" << std::endl;
                    begin = ros::Time::now().toSec();
                    f_keep_rrt = true;
                }
                if (ros::Time::now().toSec() - begin > 3.0)  {
                    best_idx += 1;
                    if (best_idx > best_goal.size())    {
                        best_idx = 0;
                        // std::cout << "no available path!!!!!" << std::endl;
                        ROS_ERROR("no available path!!!!!");
                        return;
                    }
                }
            }   
        // }
    }
    else    {
        // if (f_isDirect) {
        //     if(!pg_->checkDirectPath(g_current_position, g_current_orientation, 
        //                         waypoints.end()->g_pos, q))    {
        //         f_control = false;
        //     }
        // }
        // else 
        if (waypoints.size() % 15 == 0)   {
            std::cout << "long way replan !" << std::endl;
            f_control = false;  // replan
        }
        else {
            for (iter = waypoints.begin(); iter != waypoints.end(); ++iter) {
                if (pg_->checkPoint(iter->g_pos)) {
                    std::cout << "collision replan !" << std::endl;
                    best_idx += 1;
                    f_control = false;  // replan
                    break;
                }
            }
        }
    }
    if(p_verbose)   {
        v_voxels(new_voxels);
        v_frontiers(false); // spatial frontiers
        v_frontiers(true);  // surface frontiers
        v_rrt_path();
    }
}