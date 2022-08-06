#ifndef UTILITY_H
#define UTILITY_H

#include <chrono> 
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

///// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/tf.h>
///// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <signal.h>

using namespace std::chrono; 
using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////
//// handler
////////////////////////////////////////////////////////////////////////////////////////////////
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////
//// Timer
////////////////////////////////////////////////////////////////////////////////////////////////
high_resolution_clock::time_point start; //global, to use tic()
void tic(){
   start = high_resolution_clock::now();
}
void toc(){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%.3f ms spent", duration.count()/1000.0);
}
void toc(string text){
   auto stop = high_resolution_clock::now();
   auto duration = duration_cast<microseconds>(stop - start);
   // cout << duration.count()/1000.0 << " ms spent" << endl;
   ROS_INFO("%s %.3f ms spent", text.c_str(), duration.count()/1000.0);
}

////////////////////////////////////////////////////////////////////////////////////////////////
//// PCL msg type conversion
////////////////////////////////////////////////////////////////////////////////////////////////
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<pcl::PointXYZ> cloud, std::string frame_id = "camera_link")
{
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

pcl::PointCloud<pcl::PointXYZ> cloudmsg2cloud(sensor_msgs::PointCloud2 cloudmsg)
{
  pcl::PointCloud<pcl::PointXYZ> cloudresult;
  pcl::fromROSMsg(cloudmsg,cloudresult);
  return cloudresult;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//// Eigen, Geo_msg conversion
////////////////////////////////////////////////////////////////////////////////////////////////
geometry_msgs::Pose eigen2geoPose(Eigen::Matrix4f pose) {
    geometry_msgs::Pose geoPose;

    tf::Matrix3x3 m;
    m.setValue((double)pose(0,0),
            (double)pose(0,1),
            (double)pose(0,2),
            (double)pose(1,0),
            (double)pose(1,1),
            (double)pose(1,2),
            (double)pose(2,0),
            (double)pose(2,1),
            (double)pose(2,2));

    tf::Quaternion q;
    m.getRotation(q);
    geoPose.orientation.x = q.getX();
    geoPose.orientation.y = q.getY();
    geoPose.orientation.z = q.getZ();
    geoPose.orientation.w = q.getW();

    geoPose.position.x = pose(0,3);
    geoPose.position.y = pose(1,3);
    geoPose.position.z = pose(2,3);

    return geoPose;
}

Eigen::Matrix4f geoPose2eigen(geometry_msgs::Pose geoPose) {
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  tf::Quaternion q(geoPose.orientation.x, geoPose.orientation.y, geoPose.orientation.z, geoPose.orientation.w);
  tf::Matrix3x3 m(q);
  result(0,0) = m[0][0];
  result(0,1) = m[0][1];
  result(0,2) = m[0][2];
  result(1,0) = m[1][0];
  result(1,1) = m[1][1];
  result(1,2) = m[1][2];
  result(2,0) = m[2][0];
  result(2,1) = m[2][1];
  result(2,2) = m[2][2];
  result(3,3) = 1;

  result(0,3) = geoPose.position.x;
  result(1,3) = geoPose.position.y;
  result(2,3) = geoPose.position.z;

  return result;
}

////////////////////////////////////////////////////////////////////////////////////////////////
//// Depth to PCL
////////////////////////////////////////////////////////////////////////////////////////////////
void depth_img_to_pcl(const cv::Mat &depth_img, const cv::Mat &rgb_img, const double &scale_factor, const vector<double> &cam_intrinsic, pcl::PointCloud<pcl::PointXYZRGB> &cam_cvt_pcl){
  for (int i=0; i<depth_img.rows; i++){
    for (int j=0; j<depth_img.cols; j++){
      float temp_depth = 0.0;
      if (scale_factor == 1.0){
        temp_depth = depth_img.at<float>(i,j);
      }
      else if(scale_factor == 1000.0){
        temp_depth = depth_img.at<ushort>(i,j);
      }
      pcl::PointXYZRGB p3d;
      if (!std::isnan(temp_depth)){
        p3d.z = temp_depth/scale_factor; 
      }
      else{
        p3d.z = 20.0;
      }
      p3d.x = ( j - cam_intrinsic[4] ) * p3d.z / cam_intrinsic[2];
      p3d.y = ( i - cam_intrinsic[5] ) * p3d.z / cam_intrinsic[3];
      p3d.r = rgb_img.at<cv::Vec3b>(i,j)[2];
      p3d.g = rgb_img.at<cv::Vec3b>(i,j)[1];
      p3d.b = rgb_img.at<cv::Vec3b>(i,j)[0];
      cam_cvt_pcl.push_back(p3d);
    }
  }
}

////// Math ///////////
void yawSaturation(double& yaw) {
  while (yaw < -M_PI)
    yaw += 2 * M_PI;
  while (yaw > M_PI)
    yaw -= 2 * M_PI;
}

#endif