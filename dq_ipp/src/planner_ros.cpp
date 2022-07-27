#include "planner_ros.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

planner_ros_class::planner_ros_class(const ros::NodeHandle& nh,
                                    const ros::NodeHandle& nh_private)
        : planner_class(), nh_(nh), nh_private_(nh_private)  {
    // get_param();

    pub_target = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
    sub_pose = nh_.subscribe("odometry", 1, &planner_ros_class::cb_pose, this);

    ROS_INFO_STREAM("\n******************** Initialized Planner ********************\n");
}

void planner_ros_class::cb_pose(const geometry_msgs::PoseStamped& msg) {
    // Track the current pose
    current_position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    current_orientation = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

    // // check target reached 
    // if (running_ && !target_reached_) {
    //     // check goal pos reached (if tol is set)
    //     if (p_replan_pos_threshold_ <= 0 ||
    //         (target_position_ - current_position_).norm() <
    //             p_replan_pos_threshold_) {
    //         // check goal yaw reached (if tol is set)
    //         double yaw = tf::getYaw(msg.pose.orientation);
    //         if (p_replan_yaw_threshold_ <= 0 ||
    //             defaults::angleDifference(target_yaw_, yaw) <
    //                 p_replan_yaw_threshold_) {
    //             target_reached_ = true;
    //         }
    //     }
    // }
}