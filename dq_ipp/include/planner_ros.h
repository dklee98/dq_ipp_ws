#ifndef PLANNER_ROS_H
#define PLANNER_ROS_H

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

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

protected:
    // ros
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    // Subscriber, Publisher
    ros::Subscriber sub_pose;
    ros::Publisher pub_target;

    // Time
    ros::Time ros_timer;
};


#endif