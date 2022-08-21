#ifndef PLANNER_ROS_H
#define PLANNER_ROS_H

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "planner.h"

using namespace std;
using namespace std::chrono; 
using namespace Eigen;


class planner_ros_class : public planner_class{
public:
    explicit planner_ros_class(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
    virtual ~planner_ros_class() = default;

    void get_param(const ros::NodeHandle& nh);
    void pub_command_point();
    // ros callbacks
    void cb_pose(const geometry_msgs::PoseStamped& msg);
    bool cb_srv_run_planner(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    void cb_timer_frontier(const ros::TimerEvent& e);
    void cb_timer_controller(const ros::TimerEvent& e);
    void cb_timer_fsm(const ros::TimerEvent& e);

    // controller
    void init_controller();
    void pid_controller();

protected:
    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    // Subscriber, Publisher
    ros::Subscriber sub_pose;
    ros::Publisher pub_cmd_vel;
    ros::Publisher v_pub_visible_voxels;
    ros::Publisher v_pub_ftrs_spatial;
    ros::Publisher v_pub_ftrs_surface;
    ros::Publisher v_pub_rrt_path;
    ros::ServiceServer srv_run_planner;

    ros::Timer timer_frontier;
    ros::Timer timer_controller;
    ros::Timer timer_fsm;


    // Controller
    bool f_init_controller;
    bool f_keep_i, f_keep_c;
    double x_err, y_err, z_err, yaw_err;
    double Kp;
    double Kp_y;
    double p_vel_bound, p_yaw_bound;
    double p_reached_pos_th;
    
    double p_init_rot_time;
    Eigen::Vector3d keep_pos;
    Eigen::Quaterniond keep_ori;

    //visualization
    int marker_len = 0;
    std::list<int> id_list;

    void v_voxels(std::vector<Eigen::Vector3d> voxels) override;
    void v_frontiers(bool isSurface) override;
    void v_rrt_path() override;

};


#endif