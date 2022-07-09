#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_py as tf2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud

import numpy as np
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

class converter():
    def __init__(self):
        rospy.init_node('tf_converter', anonymous=True)

        self.sub_points = rospy.Subscriber('/dji_matrice_100/camera/depth/points', PointCloud2, self.points_cb)
        self.pub_points = rospy.Publisher('/dji_matrice_100/world_points', PointCloud2, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rate = rospy.Rate(30)

        

    
    def points_cb(self, msg):

        trans1 = self.tf_buffer.lookup_transform("base_link", "camera_link", rospy.Time(0), rospy.Duration(10))
        trans2 = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(10))
        
        self.world_points = do_transform_cloud(msg, trans1)
        self.world_points = do_transform_cloud(self.world_points, trans2)
        # self.world_points.header.frame_id = "world"

        self.pub_points.publish(self.world_points)
        return

if __name__ == '__main__':
    con = converter()
    while 1:
        try:
            con.rate.sleep()
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)
            


# if __name__ == '__main__':
#     rospy.init_node('tf_converter')

#     listener = tf.TransformListener()

#     pub_converted_pc = rospy.Publisher('/dji_matrice_100/world_points', PointCloud2, queue_size=10)

#     rate = rospy.Rate(30.0)
#     while not rospy.is_shutdown():
#         try:
#             (t_body_t_cam,r_body_t_cam) = listener.lookupTransform('base_link', 'camera_link', rospy.Time(0))
#             (t_map_t_body,r_map_t_body) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             continue

#         angular = 4 * math.atan2(trans[1], trans[0])
#         linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
#         cmd = geometry_msgs.msg.Twist()
#         cmd.linear.x = linear
#         cmd.angular.z = angular
#         turtle_vel.publish(cmd)

        # rate.sleep()

# #!/usr/bin/env python
# import rospy
# # from gazebo_msgs.msg import ModelStates
# from sensor_msgs.msg import PointCloud2
# from tf2_msgs.msg import TFMessage
# import tf

# class tf_converter():
#     def __init__(self):
#         rospy.init_node('tf_converter', anonymous=True)
#         self.camera_link_name = rospy.get_param("/camera_link_name", 'camera_link')
#         # self.raw_pointcloud_name = rospy.get_param("/raw_pointcloud_name", '/dji_matrice_100/camera/depth/points')
#         # self.base_link_name = rospy.get_param("/base_link_name", 'base_link')
    
#         self.pub_world_data = rospy.Publisher('/dji_matrice_100/world_points', PointCloud2, queue_size=10)
        
#         self.sub_tf= rospy.Subscriber('/tf', TFMessage, self.tf_cb)
#         self.sub_raw_data = rospy.Subscriber('/dji_matrice_100/camera/depth/points', PointCloud2, self.base_cb)
#         self.rate = rospy.Rate(30)

#         self.T_body_t_cam_check = False
#         # self.base_check=0
#         # self.br = tf.TransformBroadcaster()

#     def tf_cb(self, msg):
#         for tf_in in msg.transforms:
#             if (tf_in.child_frame_id == self.camera_link_name) and not self.T_body_t_cam_check:
#                 m = tf.transformations.quaternion_matrix((tf_in.transform.rotation.x, tf_in.transform.rotation.y, \
#                                                         tf_in.transform.rotation.z, tf_in.transform.rotation.w))
#                 self.T_body_t_cam[0][0:3] = m[0][0:3]
#                 self.T_body_t_cam[1][0:3] = m[1][0:3]
#                 self.T_body_t_cam[2][0:3] = m[2][0:3]
#                 self.T_body_t_cam[0][3] = tf_in.transform.translation.x
#                 self.T_body_t_cam[1][3] = tf_in.transform.translation.y
#                 self.T_body_t_cam[2][3] = tf_in.transform.translation.z
#                 self.T_body_t_cam[3][3] = 1.0
#                 self.T_body_t_cam_check = True

#             elif tf_in.child_frame_id == "base_link":
#                 mm = tf.transformations.quaternion_matrix((tf_in.transform.rotation.x, tf_in.transform.rotation.y, \
#                                                         tf_in.transform.rotation.z, tf_in.transform.rotation.w))
#                 print(mm)
#                 print("==========")
#                 print(self.T_map_t_body)
#                 print("-----------")
#                 self.T_map_t_body[0][0:3] = mm[0][0:3]
#                 print(self.T_map_t_body)
#                 print("-----------")
#                 self.T_map_t_body[1][0:3] = mm[1][0:3]
#                 print(self.T_map_t_body)
#                 print("-----------")
#                 self.T_map_t_body[2][0:3] = mm[2][0:3]
#                 print(self.T_map_t_body)
#                 print("-----------")
#                 self.T_map_t_body[0][3] = tf_in.transform.translation.x
#                 print(self.T_map_t_body)
#                 print("-----------")
#                 self.T_map_t_body[1][3] = tf_in.transform.translation.y
#                 print(self.T_map_t_body)
#                 print("-----------")
#                 self.T_map_t_body[2][3] = tf_in.transform.translation.z
#                 print(self.T_map_t_body)
#                 print("-----------")
#                 self.T_map_t_body[3][3] = 1.0
#                 print(self.T_map_t_body)
#                 print("#######################################")

#     def base_cb(self, msg):
#         # self.base_check=1
#         self.header = msg.header.stamp
        
#         return

# if __name__ == '__main__':
#     con = tf_converter()
#     while 1:
#         try:
#             con.rate.sleep()
#         except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
#             sys.exit(0)