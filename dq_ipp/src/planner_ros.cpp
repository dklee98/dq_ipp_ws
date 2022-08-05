#include "planner_ros.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

planner_ros_class::planner_ros_class(const ros::NodeHandle& nh,
                                    const ros::NodeHandle& nh_private)
        : planner_class(nh), nh_(nh), nh_private_(nh_private)  {
    // get_param();

    sub_pose = nh_.subscribe("odometry", 1, &planner_ros_class::cb_pose, this);
    pub_target = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
    v_pub_visible_voxels = nh_.advertise<visualization_msgs::MarkerArray>("visualization/visible_voxels", 1);
    // v_pub_surface_frontiers = nh_.advertise<visualization_msgs::MarkerArray>("visualization/surface_frontiers", 1);
    // v_pub_spatial_frontiers = nh_.advertise<visualization_msgs::MarkerArray>("visualization/spatial_frontiers", 1);

    v_ftrs_clusters = nh_.advertise<visualization_msgs::MarkerArray>("visualization/clustered_frontiers", 1);

    timer_frontier = nh_.createTimer(ros::Duration(0.5), &planner_ros_class::cb_timer_frontier, this);

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

void planner_ros_class::cb_timer_frontier(const ros::TimerEvent& e)    {
    if (!f_planning)    {
        ROS_INFO("\n******************** Planner stopped ********************\n");
        return;
    }
    test();
}

void planner_ros_class::v_voxels(std::vector<Eigen::Vector3d> voxels) {
    visualization_msgs::MarkerArray arr_marker;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.type = visualization_msgs::Marker::CUBE_LIST;
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = c_voxel_size;
    marker.pose.orientation.w = 1.0;

    Eigen::Vector3d bmin = voxels.front();
    Eigen::Vector3d bmax = voxels.front();
    
    for (int i = 0; i < voxels.size(); ++i) {
        for (int j = 0; j < 3; ++j) {
            bmin[j] = min(bmin[j], voxels[i][j]);
            bmax[j] = max(bmax[j], voxels[i][j]);
        }
        geometry_msgs::Point cube_center;
        cube_center.x = voxels[i][0];
        cube_center.y = voxels[i][1];
        cube_center.z = voxels[i][2];
        marker.points.push_back(cube_center);
        std_msgs::ColorRGBA color_msg;
        color_msg.g = 0.5;
        color_msg.a = 0.5;
        marker.colors.push_back(color_msg);
    }
    arr_marker.markers.push_back(marker);
    // bbox
    visualization_msgs::Marker b_marker;
    b_marker.header.frame_id = "world";
    b_marker.header.stamp = ros::Time::now();
    b_marker.type = visualization_msgs::Marker::CUBE;
    b_marker.id = 1;
    b_marker.action = visualization_msgs::Marker::ADD;
    b_marker.scale.x = bmax[0] - bmin[0];
    b_marker.scale.y = bmax[1] - bmin[1];
    b_marker.scale.z = bmax[2] - bmin[2];
    b_marker.pose.orientation.w = 1.0;
    b_marker.pose.position.x = (bmin[0] + bmax[0]) / 2;
    b_marker.pose.position.y = (bmin[1] + bmax[1]) / 2;
    b_marker.pose.position.z = (bmin[2] + bmax[2]) / 2;
    b_marker.color.b = 0.5;
    b_marker.color.a = 0.5;
    arr_marker.markers.push_back(b_marker);
    v_pub_visible_voxels.publish(arr_marker);
}

void planner_ros_class::v_frontiers() {
    visualization_msgs::MarkerArray arr_marker;
    int sampler = 50;
    for (auto& ftr : ft_->frontiers) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.id = ftr.id_;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = c_voxel_size;
        marker.pose.orientation.w = 1.0;
        for (auto& cell : ftr.cells_)  {
            geometry_msgs::Point cube_center;
            cube_center.x = cell[0];
            cube_center.y = cell[1];
            cube_center.z = cell[2];
            marker.points.push_back(cube_center);
            std_msgs::ColorRGBA color_msg;
            srand(ftr.id_);
            if (ftr.id_ % 3 == 0)
                color_msg.r = 0 + (float)(rand()) / ((float)(RAND_MAX/(1-0)));
            else if (ftr.id_ % 3 == 1)
                color_msg.g = 0 + (float)(rand()) / ((float)(RAND_MAX/(1-0)));
            else if (ftr.id_ % 3 == 2)
                color_msg.b = 0 + (float)(rand()) / ((float)(RAND_MAX/(1-0)));
            color_msg.a = 0.5;
            marker.colors.push_back(color_msg);
        }
        geometry_msgs::Point cube_avg;
        cube_avg.x = ftr.average_[0];
        cube_avg.y = ftr.average_[1];
        cube_avg.z = ftr.average_[2];
        marker.points.push_back(cube_avg);
        std_msgs::ColorRGBA color_msg;
        color_msg.r = 1.0;
        color_msg.g = 1.0;
        color_msg.b = 1.0;
        color_msg.a = 1.0;
        marker.colors.push_back(color_msg);

        arr_marker.markers.push_back(marker);
    }
    v_ftrs_clusters.publish(arr_marker);
}



