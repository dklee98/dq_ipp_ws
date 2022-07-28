#include "planner_ros.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

planner_ros_class::planner_ros_class(const ros::NodeHandle& nh,
                                    const ros::NodeHandle& nh_private)
        : planner_class(), nh_(nh), nh_private_(nh_private)  {
    // get_param();

    sub_pose = nh_.subscribe("odometry", 1, &planner_ros_class::cb_pose, this);
    pub_target = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
    v_pub_visible_voxels = nh_.advertise<visualization_msgs::MarkerArray>("visualization/visible_voxels", 1);
    v_pub_surface_frontiers = nh_.advertise<visualization_msgs::MarkerArray>("visualization/surface_frontiers", 1);
    v_pub_spatial_frontiers = nh_.advertise<visualization_msgs::MarkerArray>("visualization/spatial_frontiers", 1);

    srv_run_planner = nh_private_.advertiseService("toggle_running", &planner_ros_class::cb_srv_run_planner, this);

    ROS_INFO_STREAM("\n******************** Initialized Planner ********************\n");
}

void planner_ros_class::cb_pose(const geometry_msgs::PoseStamped& msg) {
    // Track the current pose
    g_current_position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    g_current_orientation = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);

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

bool planner_ros_class::cb_srv_run_planner(std_srvs::SetBool::Request& req, 
                                            std_srvs::SetBool::Response& res)   {
    res.success = true;
    if (req.data)   {
        f_planning = true;
        ROS_INFO("Started planning.");
    }
    else{
        f_planning = false;
        ROS_INFO("Stopped planning.");
    }
    return true;
}

void planner_ros_class::planning_loop() {
    // This is the main loop, spinning is managed explicitely for efficiency
    ROS_INFO("\n******************** Planner is now Running ********************\n");
    while(ros::ok())    {
        if (f_planning) {
            test();
        }
        ros::spinOnce();
    }
}

void planner_ros_class::v_voxels(std::vector<Eigen::Vector3d> voxels, 
                                double voxel_size,
                                int print_type) {
    visualization_msgs::MarkerArray arr_marker;
    for (int i = 0; i < voxels.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CUBE;
        marker.id = i;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = voxel_size;
        marker.pose.orientation.w = 1.0;
        marker.pose.position.x = voxels[i][0];
        marker.pose.position.y = voxels[i][1];
        marker.pose.position.z = voxels[i][2];
        if (print_type == V_visible_voxels)   {
            marker.color.g = 0.5;
            marker.color.a = 1.0;
        }
        else if (print_type == V_surface_frontiers)   {
            marker.color.r = 0.5;
            marker.color.a = 1.0;
        }
        else if (print_type == V_spatial_frontiers)   {
            marker.color.b = 0.5;
            marker.color.a = 1.0;
        }
        marker.color.g = 0.5;
        marker.color.a = 1.0;
        arr_marker.markers.push_back(marker);
        if(i > 1000)        {
            arr_marker.markers.erase(arr_marker.markers.begin());
        }
    }
    if (print_type == V_visible_voxels)   {
        v_pub_visible_voxels.publish(arr_marker);
        ROS_INFO("len visible markers : %6d", arr_marker.markers.size());
    }
    else if (print_type == V_surface_frontiers)   {
        v_pub_surface_frontiers.publish(arr_marker);
        ROS_INFO("len surface markers : %6d", arr_marker.markers.size());
    }
    else if (print_type == V_spatial_frontiers)   {
        v_pub_spatial_frontiers.publish(arr_marker);
        ROS_INFO("len spatial markers : %6d", arr_marker.markers.size());
    }
}