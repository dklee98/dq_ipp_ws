#include "path_generator.h"

using namespace std;

path_generator_class::path_generator_class(voxblox_class& map, const ros::NodeHandle& nh) : map_(map)  {
    get_param(nh);
    rrt_star_ = make_shared<RRT_STAR>(p_max_rrt_iteration, p_rrt_collision_r, p_extension_range, p_map_min, p_map_max);
}

void path_generator_class::get_param(const ros::NodeHandle& nh)    {
    nh.param("/planner_node/max_rrt_iteration", p_max_rrt_iteration, 10000);
    nh.param("/planner_node/rrt_collision_r", p_rrt_collision_r, 0.5);
    nh.param("/planner_node/extension_range", p_extension_range, 1.0);
    nh.param("/planner_node/goal_reached_tolerance_distance", p_goal_reached_tolerance_distance, 1.0);
    nh.getParam("/planner_node/map_min", p_map_min);
    nh.getParam("/planner_node/map_max", p_map_max);
}

nav_msgs::Path path_generator_class::ex_func_uav_pose(const Eigen::Vector3d& cur_pos, 
                                                    const Eigen::Quaterniond& cur_ori,
                                                    const Eigen::Vector3d& goal_pos,
                                                    const Eigen::Quaterniond& goal_ori)  {
    return rrt_star_->execute(map_.esdf_server, cur_pos, goal_pos, cur_ori, goal_ori, 
                                p_goal_reached_tolerance_distance);
}