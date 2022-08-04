#ifndef PLANNER_ROS_H
#define PLANNER_ROS_H

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 

#include <geometry_msgs/PoseStamped.h>
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

    // void get_param();

    // ros callbacks
    void cb_pose(const geometry_msgs::PoseStamped& msg);
    bool cb_srv_run_planner(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

    void cb_timer_frontier(const ros::TimerEvent& e);

    // planning loop
    // void planning_loop();

protected:
    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    // Subscriber, Publisher
    ros::Subscriber sub_pose;
    ros::Publisher pub_target;
    ros::Publisher v_pub_visible_voxels;
    ros::Publisher v_pub_surface_frontiers;
    ros::Publisher v_pub_spatial_frontiers;
    ros::Publisher v_ftrs_clusters;
    ros::ServiceServer srv_run_planner;

    ros::Timer timer_frontier;

    // Time
    ros::Time ros_timer;

    void v_voxels(std::vector<Eigen::Vector3d> voxels, double voxel_size, int print_type) override;
    void v_frontiers() override;

};


#endif