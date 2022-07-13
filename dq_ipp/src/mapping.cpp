#include "mapping.h"

using namespace std;
using namespace std::chrono;
using namespace Eigen;

mapping_class::mapping_class(const ros::NodeHandle& n) : nh(n)  
{
    ROS_WARN("Class generating...");
    get_param();

    // Transformation body to cam
    m_body_t_cam.block<3, 3>(0, 0) = Quaterniond(-0.5, 0.5, -0.5, 0.5).toRotationMatrix();
    m_body_t_cam.block<3, 1>(0, 3) = Vector3d(m_cam_extrinsic[0], m_cam_extrinsic[1], m_cam_extrinsic[2]);

    // Subscriber
    sub_pose = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 3, &mapping_class::cb_pose, this);

    // Publisher
    pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("/aligned_points", 10);
    
    // sync subscriber
    static message_filters::Subscriber<sensor_msgs::Image> sub_depth;
	static message_filters::Subscriber<sensor_msgs::Image> sub_rgb;
    sub_depth.subscribe(nh, m_depth_topic, 5);
    sub_rgb.subscribe(nh, m_rgb_topic, 5);
    static message_filters::Synchronizer<rgbd_sync_policy> sub_rgbd_sync(rgbd_sync_policy(5), sub_depth, sub_rgb);
    sub_rgbd_sync.registerCallback(&mapping_class::cb_rgbd, this);
    
}

mapping_class::~mapping_class(){}

void mapping_class::get_param()    
{
    nh.param<std::string>("/depth_topic_name", m_depth_topic, "/dji_matrice_100/camera/depth/image_raw");
    nh.param<std::string>("/rgb_topic_name", m_rgb_topic, "/dji_matrice_100/camera/rgb/image_raw");
    
    nh.getParam("/cam_extrinsic", m_cam_extrinsic);
    nh.getParam("/cam_intrinsic", m_cam_intrinsic);
}

void mapping_class::cb_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)    {
    geometry_msgs::Pose pose = msg->pose;
    Matrix4d tmp_mat = Matrix4d::Identity();

    tmp_mat.block<3, 3>(0, 0) = Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z).toRotationMatrix();
    tmp_mat.block<3, 1>(0, 3) = Vector3d(pose.position.x, pose.position.y, pose.position.z);

    m_map_t_body = tmp_mat;
    {
        lock_guard<mutex> lock(m_mutex_pose);
        m_pose_input = make_pair(msg->header.stamp.toSec(), tmp_mat);
    }
    m_pose_check=true;
}

void mapping_class::cb_rgbd(const sensor_msgs::Image::ConstPtr& depth_msg, const sensor_msgs::Image::ConstPtr& rgb_msg){
	if (m_pose_check){
		m_downsampling_counter++;

		Matrix4d map_t_cam;
		bool is_time_sync=false;
        cv::Mat depth_img, rgb_img;
        try {
            if (depth_msg->encoding=="32FC1"){
                cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(*depth_msg, "32FC1"); // == sensor_msgs::image_encodings::TYPE_32FC1
                depth_img = depth_ptr->image;
                m_scale_factor=1.0;
            }
            else if (depth_msg->encoding=="16UC1"){ // uint16_t (stdint.h) or ushort or unsigned_short
                cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(*depth_msg, "16UC1"); // == sensor_msgs::image_encodings::TYPE_16UC1
                depth_img = depth_ptr->image;
                m_scale_factor=1000.0;
            }

            cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(*rgb_msg, sensor_msgs::image_encodings::BGR8);
            rgb_img = img_ptr->image;
            {
                lock_guard<mutex> lock(m_mutex_pose);
                if (fabs(m_pose_input.first - depth_msg->header.stamp.toSec()) < 0.03){
                    map_t_cam = m_pose_input.second * m_body_t_cam; // map_t_body * body_t_cam
                    is_time_sync=true;
                }
            }
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Error to cvt depth img");
            return;
        }


        //// down sampling
        if (m_downsampling_counter % m_downsampling_devider == 0 && is_time_sync){
            pcl::PointCloud<pcl::PointXYZRGB> cam_cvt_pcl;
            depth_img_to_pcl(depth_img, rgb_img, m_scale_factor, m_cam_intrinsic, cam_cvt_pcl);
        
        // publish aligned pointcloud (voxblox input & visualization)
        sensor_msgs::PointCloud2 cloud_ros;
        pcl::toROSMsg(cam_cvt_pcl, cloud_ros);
        cloud_ros.header.stamp = depth_msg->header.stamp;
        cloud_ros.header.frame_id = "camera_link";
        pub_pointcloud.publish(cloud_ros);


        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr before_voxelize(new pcl::PointCloud<pcl::PointXYZRGB>());
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr after_voxelize(new pcl::PointCloud<pcl::PointXYZRGB>());
        // *before_voxelize = cam_cvt_pcl;
        // m_voxelgrid.setInputCloud(before_voxelize);
        // m_voxelgrid.filter(*after_voxelize);
        // cam_cvt_pcl = *after_voxelize;

        //     m_tsdfesdf_voxblox->insertPointcloud(cam_cvt_pcl, map_t_cam.cast<float>());
        //     m_tsdfesdf_voxblox->updateMesh();
        //     voxblox_msgs::Mesh mesh_msg;
        // m_tsdfesdf_voxblox->getTsdfMeshForPublish(mesh_msg);
        // mesh_msg.header.frame_id = "map";
        // m_voxblox_mesh_pub.publish(mesh_msg);
        }

	}
}