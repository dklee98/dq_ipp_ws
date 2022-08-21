#include "planner_ros.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

planner_ros_class::planner_ros_class(const ros::NodeHandle& nh,
                                    const ros::NodeHandle& nh_private)
        : planner_class(nh), nh_(nh), nh_private_(nh_private)  {
    get_param(nh);

    sub_pose = nh_.subscribe("odometry", 1, &planner_ros_class::cb_pose, this);
    pub_cmd_vel = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);

    v_pub_visible_voxels = nh_.advertise<visualization_msgs::MarkerArray>("visualization/visible_voxels", 1);
    v_pub_ftrs_spatial = nh_.advertise<visualization_msgs::MarkerArray>("visualization/spatial_frontiers", 1);
    v_pub_ftrs_surface = nh_.advertise<visualization_msgs::MarkerArray>("visualization/surface_frontiers", 1);
    v_pub_rrt_path = nh_.advertise<nav_msgs::Path>("visualization/rrt_path", 1);

    timer_frontier = nh_.createTimer(ros::Duration(0.5), &planner_ros_class::cb_timer_frontier, this);
    timer_controller = nh_.createTimer(ros::Duration(0.05), &planner_ros_class::cb_timer_controller, this);
    timer_fsm = nh_.createTimer(ros::Duration(0.01), &planner_ros_class::cb_timer_fsm, this);

    srv_run_planner = nh_private_.advertiseService("toggle_running", &planner_ros_class::cb_srv_run_planner, this);

    ROS_INFO_STREAM("\n******************** Initialized Planner ********************\n");
}

void planner_ros_class::get_param(const ros::NodeHandle& nh)    {
    nh.param("/planner_node/p_gain", Kp, 0.5);
    nh.param("/planner_node/p_gain_yaw", Kp_y, 0.5);
    nh.param("/planner_node/linear_vel_bound", p_vel_bound, 1.0);
    nh.param("/planner_node/yaw_bound", p_yaw_bound, 0.8);
    nh.param("/planner_node/init_rot_sec", p_init_rot_time, 12.0);
    nh.param("/planner_node/reached_pos_th", p_reached_pos_th, 0.2);
    f_init_controller = false;
    f_keep_i = false;
    f_keep_c = false;
}

void planner_ros_class::cb_pose(const geometry_msgs::PoseStamped& msg) {
    {
        lock_guard<mutex> lock(m_mutex);
        // Track the current pose
        g_current_position = Eigen::Vector3d(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
        g_current_orientation = Eigen::Quaterniond(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    }
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

void planner_ros_class::cb_timer_controller(const ros::TimerEvent& e)    {
    if (!f_init_controller && f_planning)    {
        init_controller();
        return;
    }
    else if (f_planning){
        std::cout << "--" << std::endl;
        pid_controller();
    }
}

void planner_ros_class::cb_timer_fsm(const ros::TimerEvent& e)    {
    if (!f_planning)    {
        return;
    }
    else if (f_planning){
        // fsm();
    }
}


void planner_ros_class::init_controller()   {
    if (!f_keep_i)    {
        begin = ros::Time::now().toSec();
        keep_pos = g_current_position;
        keep_ori = g_current_orientation;
        f_keep_i = true;
    }
    if (ros::Time::now().toSec() - begin > 2 * p_init_rot_time)  {
        f_init_controller = true;
    }
    x_err = keep_pos[0] - g_current_position[0];
    y_err = keep_pos[1] - g_current_position[1];
    z_err = keep_pos[2] - g_current_position[2];

    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.twist.linear.x = bound(Kp * x_err, p_vel_bound);
    cmd_vel.twist.linear.y = bound(Kp * y_err, p_vel_bound);
    cmd_vel.twist.linear.z = bound(Kp * z_err, p_vel_bound);
    cmd_vel.twist.angular.z = 360/p_init_rot_time * M_PI/180;
    pub_cmd_vel.publish(cmd_vel);
}

void planner_ros_class::pid_controller()    {
    if (!f_control) {
        if (!f_keep_c)  {
            begin = ros::Time::now().toSec();
            keep_pos = g_current_position;
            keep_ori = g_current_orientation;
            f_keep_c = true;
        }
        // 시간 오바되면 goal 바꾸게 설정 하기!

        x_err = keep_pos[0] - g_current_position[0];
        y_err = keep_pos[1] - g_current_position[1];
        z_err = keep_pos[2] - g_current_position[2];

        geometry_msgs::TwistStamped cmd_vel;
        cmd_vel.header.stamp = ros::Time::now();
        cmd_vel.twist.linear.x = bound(Kp * x_err, p_vel_bound);
        cmd_vel.twist.linear.y = bound(Kp * y_err, p_vel_bound);
        cmd_vel.twist.linear.z = bound(Kp * z_err, p_vel_bound);
        pub_cmd_vel.publish(cmd_vel);
        return;
    }
    
    if (waypoints.size() == 0)  {
        lock_guard<mutex> lock(m_mutex);
        f_control = false;
        f_keep_c = false;
        return;
    }
    x_err = waypoints.front().g_pos[0] - g_current_position[0];
    y_err = waypoints.front().g_pos[1] - g_current_position[1];
    z_err = waypoints.front().g_pos[2] - g_current_position[2];
    
    double yaw_goal = waypoints.front().g_yaw;
    tf::Matrix3x3 m_tmp_;
    tf::matrixEigenToTF(g_current_orientation.toRotationMatrix(), m_tmp_);
    double r_, p_, yaw_cur;
    m_tmp_.getRPY(r_, p_, yaw_cur);   
    yaw_err = yaw_goal - yaw_cur;
    yawSaturation(yaw_err);
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.twist.linear.x = bound(Kp * x_err, p_vel_bound);
    cmd_vel.twist.linear.y = bound(Kp * y_err, p_vel_bound);
    cmd_vel.twist.linear.z = bound(Kp * z_err, p_vel_bound);
    cmd_vel.twist.angular.z = bound(Kp_y * yaw_err, p_yaw_bound);
    pub_cmd_vel.publish(cmd_vel);

    int cnt = 0;
    for (iter = waypoints.begin(); iter != waypoints.end(); ++iter)  {
        if (iter == waypoints.end() - 1)    {
            if ((g_current_position - iter->g_pos).norm() < 0.3)    {
                std::cout << "last node" << std::endl;
                cnt += 1;
            }
        }
        else if ((g_current_position - iter->g_pos).norm() < p_reached_pos_th)    {
            cnt += 1;
        }
    }
    if (cnt > 0)    {
        for (int i = 0; i < cnt; ++i)   {
            waypoints.pop_front();
        }
    }
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
    visualization_msgs::MarkerArray del_markers;
    for (int i = 0; i < marker_len; ++i)  {
        visualization_msgs::Marker del_marker;
        del_marker.header.frame_id = 'world';
        del_marker.header.stamp = ros::Time::now();
        del_marker.action = visualization_msgs::Marker::DELETE;
        del_marker.id = i;
        del_markers.markers.push_back(del_marker);
    }
    if (isSurface) v_pub_ftrs_surface.publish(del_markers);
    else v_pub_ftrs_spatial.publish(del_markers);

    marker_len = 0;
    visualization_msgs::MarkerArray arr_marker;
    std::list<Frontier> frontiers;
    if (isSurface)  frontiers = ft_->surface_frontiers;
    else  frontiers = ft_->spatial_frontiers;

    for (auto& ftr : frontiers) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.id = ++marker_len;
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
        visualization_msgs::Marker vp_marker, nv_marker, vp_txt_1, vp_txt_2;
        vp_marker.header.frame_id = nv_marker.header.frame_id = vp_txt_1.header.frame_id = vp_txt_2.header.frame_id = "world";
        vp_marker.header.stamp = nv_marker.header.stamp = vp_txt_1.header.stamp = vp_txt_2.header.stamp = ros::Time::now();
        vp_marker.type = visualization_msgs::Marker::ARROW; 
        nv_marker.type = visualization_msgs::Marker::LINE_LIST; 
        vp_txt_1.type = vp_txt_2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        vp_marker.id = ++marker_len;
        nv_marker.id = ++marker_len;
        vp_txt_1.id = ++marker_len;
        vp_txt_2.id = ++marker_len;
        vp_marker.action = nv_marker.action = vp_txt_1.action = vp_txt_2.action = visualization_msgs::Marker::ADD;
        vp_marker.scale.x = 0.5; nv_marker.scale.x = 0.05;
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
        // normal vector marker
        geometry_msgs::Point p;
        p.x = ftr.average_[0]; p.y = ftr.average_[1]; p.z = ftr.average_[2];
        nv_marker.points.push_back(p);
        p.x += ftr.normal[0]; p.y += ftr.normal[1]; p.z += ftr.normal[2];
        nv_marker.points.push_back(p);
        p.x = ftr.average_[0]; p.y = ftr.average_[1]; p.z = ftr.average_[2];
        nv_marker.points.push_back(p);
        p.x += ftr.tangent[0]; p.y += ftr.tangent[1]; p.z += ftr.tangent[2];
        nv_marker.points.push_back(p);
        nv_marker.pose.orientation.w = 1.0;
        // text
        vp_txt_1.text = vp_txt_2.text = std::to_string(ftr.id_);
        // color
        vp_marker.color.r = nv_marker.color.r = 1.0; vp_txt_1.color.r = vp_txt_2.color.r = 0.0;
        vp_marker.color.g = nv_marker.color.g = 1.0; vp_txt_1.color.g = vp_txt_2.color.g = 0.0;
        vp_marker.color.b = nv_marker.color.b = 1.0; vp_txt_1.color.b = vp_txt_2.color.b = 0.0;
        vp_marker.color.a = nv_marker.color.a = vp_txt_1.color.a = vp_txt_2.color.a = 1.0;
        // push
        arr_marker.markers.push_back(vp_marker);
        arr_marker.markers.push_back(nv_marker);
        arr_marker.markers.push_back(vp_txt_1);
        arr_marker.markers.push_back(vp_txt_2);

    }
    if (isSurface) v_pub_ftrs_surface.publish(arr_marker);
    else v_pub_ftrs_spatial.publish(arr_marker);
}

void planner_ros_class::v_rrt_path() {
    v_pub_rrt_path.publish(rrt_result);
}
