#ifndef MAPPING_H
#define MAPPING_H

#include "utility.hpp"

#include <ros/ros.h>
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
#include <iostream> //cout
#include <math.h> // pow, atan2
#include <chrono> 

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

// callback two topics at once
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace std::chrono; 
using namespace Eigen;


class mapping_class{
private:
    ros::NodeHandle nh;

    std::string m_depth_topic;
    std::string m_rgb_topic;

    ros::Subscriber sub_pose;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> rgbd_sync_policy;
    ros::Publisher pub_pointcloud;

    vector<double> m_cam_intrinsic, m_cam_extrinsic;

    Matrix4d m_body_t_cam = Matrix4d::Identity();

    // pose callback
    mutex m_mutex_pose;
    Matrix4d m_map_t_body = Matrix4d::Identity();
    pair<double, Matrix4d> m_pose_input;
    bool m_pose_check = false;

    // rgbd callback
    double m_scale_factor = 1.0;
    int m_downsampling_counter = 0;
    int m_downsampling_devider = 5;
    cv::Mat m_current_rgb_image;

    void get_param();
    void cb_rgbd(const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::Image::ConstPtr& rgb_msg);
    void cb_pose(const geometry_msgs::PoseStamped::ConstPtr& msg);

public:
    mapping_class(const ros::NodeHandle& n);
    ~mapping_class();
};


#endif