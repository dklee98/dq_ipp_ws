#include "path_generator.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

path_generator_class::path_generator_class(voxblox_class& map, 
                                    const ros::NodeHandle& nh) : map_(map)  {
    get_param(nh);
    // TODO
}

void path_generator_class::get_param(const ros::NodeHandle& nh)    {
    // 'planner_node' namespace is necessary!!
    nh.param("/planner_node/p_parameter_name", p_parameter_name, 0.0);
    // How to use voxblox.h function
    c_voxel_size = map_.getVoxelSize();  
    
    // TODO
}

void path_generator_class::ex_func_uav_pose(const Eigen::Vector3d& cur_pos, 
                            const Eigen::Quaterniond& cur_ori)  {
    // this function is used in "planner.cpp"
    uav_cur_pos = cur_pos;  // current uav position
    uav_cur_ori = cur_ori;  // current uav orientation

    std::cout << uav_cur_pos << std::endl;

    // TODO
}

// Test
// Open 2 terminal
// $ roslaunch dq_ipp test.launch
// $ rosrun dq_ipp run_auto_planner.py
// if you use joystick, make 'take_off' then,
// $ rosservice call /planner_node/toggle_running "data: true"

// If you want to stop planner
// $ rosservice call /planner_node/toggle_running "data: false"

// Reference implementation
// voxblox.cpp
// ray_caster.cpp
// frontier.cpp