#include "planner_ros.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

planner_ros_class::planner_ros_class(const ros::NodeHandle& nh,
                                    const ros::NodeHandle& nh_private)
        : planner_class(nh), nh_(nh), nh_private_(nh_private)  {
    // get_param();

    sub_pose = nh_.subscribe("odometry", 1, &planner_ros_class::cb_pose, this);
    // pub_target = nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 10);
    pub_target = nh_.advertise<geometry_msgs::PoseStamped>("command/trajectory", 10);
    v_pub_visible_voxels = nh_.advertise<visualization_msgs::MarkerArray>("visualization/visible_voxels", 1);

    v_pub_ftrs_spatial = nh_.advertise<visualization_msgs::MarkerArray>("visualization/spatial_frontiers", 1);
    v_pub_ftrs_surface = nh_.advertise<visualization_msgs::MarkerArray>("visualization/surface_frontiers", 1);

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
    pub_command_point();
}

void planner_ros_class::pub_command_point() {
    geometry_msgs::PoseStamped output;
    output.header.frame_id = "world";
    output.header.stamp = ros::Time::now();
    output.pose.position.x = best_vp_pos[0][0];
    output.pose.position.y = best_vp_pos[0][1];
    output.pose.position.z = best_vp_pos[0][2];
    Eigen::Quaterniond q;
    ft_->yaw2orientation(best_vp_yaw[0], q);
    output.pose.orientation.x = q.x();
    output.pose.orientation.y = q.y();
    output.pose.orientation.z = q.z();
    output.pose.orientation.w = q.w();
    pub_target.publish(output);

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

void planner_ros_class::v_frontiers(bool isSurface) {
    visualization_msgs::MarkerArray arr_marker;
    std::list<Frontier> frontiers;
    if (isSurface)  frontiers = ft_->surface_frontiers;
    else  frontiers = ft_->spatial_frontiers;

    for (auto& ftr : frontiers) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.id = ftr.id_ + 1;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = c_voxel_size;
        marker.pose.orientation.w = 1.0;
        for (auto& cell : ftr.filtered_cells_)  {    // ftr.filtered_cells_ or ftr.cells_
            geometry_msgs::Point cube_center;
            cube_center.x = cell[0];
            cube_center.y = cell[1];
            cube_center.z = cell[2];
            marker.points.push_back(cube_center);
            std_msgs::ColorRGBA color_msg;
            srand(ftr.id_ * 123);
            if (ftr.id_ % 3 == 0)
                color_msg.r = 0 + (float)(rand()) / ((float)(RAND_MAX/(1-0)));
            else if (ftr.id_ % 3 == 1)
                color_msg.g = 0 + (float)(rand()) / ((float)(RAND_MAX/(1-0)));
            else if (ftr.id_ % 3 == 2)
                color_msg.b = 0 + (float)(rand()) / ((float)(RAND_MAX/(1-0)));
            color_msg.a = 0.3;
            marker.colors.push_back(color_msg);
        }
        // Visualize average point of cluster
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

        // Visualize strong viewpoint of cluster
        visualization_msgs::Marker vp_marker, vp_txt_1, vp_txt_2;
        vp_marker.header.frame_id = vp_txt_1.header.frame_id = vp_txt_2.header.frame_id = "world";
        vp_marker.header.stamp = vp_txt_1.header.stamp = vp_txt_2.header.stamp = ros::Time::now();
        vp_marker.type = visualization_msgs::Marker::ARROW; 
        vp_txt_1.type = vp_txt_2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        vp_marker.id = -(ftr.id_ + 1); 
        vp_txt_1.id = (ftr.id_ + 1) * 1000;
        vp_txt_2.id = -(ftr.id_ + 1) * 1000;
        vp_marker.action = vp_txt_1.action = vp_txt_2.action = visualization_msgs::Marker::ADD;
        vp_marker.scale.x = 0.5; 
        vp_marker.scale.y = 0.1; 
        vp_marker.scale.z = 0.1; vp_txt_1.scale.z = vp_txt_2.scale.z = 0.5;
        vp_marker.pose.position.x = vp_txt_1.pose.position.x = ftr.viewpoints_[0].pos_[0];
        vp_marker.pose.position.y = vp_txt_1.pose.position.y = ftr.viewpoints_[0].pos_[1];
        vp_marker.pose.position.z = vp_txt_1.pose.position.z = ftr.viewpoints_[0].pos_[2];
        vp_txt_2.pose.position.x = ftr.average_[0];
        vp_txt_2.pose.position.y = ftr.average_[1];
        vp_txt_2.pose.position.z = ftr.average_[2];
        Eigen::Quaterniond ori;
        ft_->yaw2orientation(ftr.viewpoints_[0].yaw_, ori);
        vp_marker.pose.orientation.x = ori.x();
        vp_marker.pose.orientation.y = ori.y();
        vp_marker.pose.orientation.z = ori.z();
        vp_marker.pose.orientation.w = ori.w();
        vp_txt_1.text = vp_txt_2.text = std::to_string(ftr.id_);
        vp_marker.color.r = 1.0; vp_txt_1.color.r = vp_txt_2.color.r = 0.0;
        vp_marker.color.g = 1.0; vp_txt_1.color.g = vp_txt_2.color.g = 0.0;
        vp_marker.color.b = 1.0; vp_txt_1.color.b = vp_txt_2.color.b = 0.0;
        vp_marker.color.a = vp_txt_1.color.a = vp_txt_2.color.a = 1.0;
        arr_marker.markers.push_back(vp_marker);
        arr_marker.markers.push_back(vp_txt_1);
        arr_marker.markers.push_back(vp_txt_2);

    }
    if (isSurface) v_pub_ftrs_surface.publish(arr_marker);
    else v_pub_ftrs_spatial.publish(arr_marker);
}
