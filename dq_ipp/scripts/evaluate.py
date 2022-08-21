#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool, SetBoolResponse
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import time
import sys
import os
import datetime
import signal
import csv

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class evaluate():
    def __init__(self):
        rospy.init_node('evaluate_node', anonymous=True)

        self.sub_tsdf = rospy.Subscriber('/planner_node/tsdf_pointcloud', PointCloud2, self.cb_tsdf)
        self.pub_debug = rospy.Publisher('/visualization/tsdf_debug', PointCloud2, queue_size = 2)
        self.pub_map_bound = rospy.Publisher('/visualization/map_bound', Marker, queue_size = 2)

        self.name = rospy.get_param("~algorithm_name", "dq_")
        self.map_min = rospy.get_param("~map_min", [-25.0, -33.0, 0.5])
        self.map_max = rospy.get_param("~map_max", [10.0, 33.0, 20.0])
        self.voxel_size = rospy.get_param("~voxel_size", 0.2)
        self.gt_building_m3 = rospy.get_param("~surface_volume", 60*0.2*68*0.2*120*0.2)
        self.gt_surface_m3 = self.gt_building_m3 - 58*0.2*66*0.2*118*0.2
        self.ground_th = rospy.get_param("~ground_height", 0.5)
        self.surface_th = rospy.get_param("~surface_th", [-0.3, 0.3])
        self.log_interval = rospy.get_param("~log_time_interval", 5)

        self.f_visualization = rospy.get_param("~visualization", False)
        self.f_verbose = rospy.get_param("~verbose", False)

        ## Setup result file
        self.eval_dir = rospy.get_param("~eval_directory", \
                        '/home/dklee/catkin_ws/src/dq_ipp_ws/results')
        if not os.path.isdir(self.eval_dir):
            rospy.logfatal("Invalid target directory '%s'.", self.eval_dir)
            sys.exit(-1)
        self.eval_dir = os.path.join(self.eval_dir, self.name +
                        datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.mkdir(self.eval_dir)
        self.result_file = open(os.path.join(self.eval_dir, "result.csv"), 'wb')
        self.writer = csv.writer(self.result_file)
        self.writer.writerow(
            ['Time[sec]', 'Volume[%]', 'Surface[%]', 'Volume[m^3]', 'Surface[m^3]'])


        self.srv_planner = rospy.Service('/planner_node/toggle_evaluate', SetBool, self.cb_srv_evaluate)
        
        self.gt_volume_m3 = (self.map_max[0]-self.map_min[0]) * \
                                (self.map_max[1]-self.map_min[1]) * \
                                (self.map_max[2]-self.map_min[2]) - self.gt_building_m3
        self.gt_surface_voxel = self.gt_surface_m3 / pow(self.voxel_size, 3)
        self.gt_voxel =  self.gt_volume_m3 / pow(self.voxel_size,3)
        self.toggle_evaluate = False
        self.f_set_timer = False

        rospy.on_shutdown(self.eval_finish)
    
    def cb_srv_evaluate(self, request):
        response = SetBoolResponse()
        response.success = True
        if request.data:
            self.toggle_evaluate = True
            self.eval_start_time = time.time()
            rospy.loginfo("Started evaluate")
        else:
            self.toggle_evaluate = False
            rospy.loginfo("Stopped evaluate")
        return response


    def cb_tsdf(self, msg):
        self.points_list = []
        self.cnt_covered_voxel = 0
        self.cnt_surface_voxel = 0
        self.covered_list = []
        self.surface_list = []
        self.v_covered = []
        self.v_surface = []
        
        for data in pc2.read_points(msg, skip_nans=True):
            self.points_list.append([data[0], data[1], data[2], data[3]])
            ## x, y, z, intensity
        print('====================')
        for point in self.points_list:
            ## ground height check
            if point[2] < self.ground_th:   
                continue
            ## map bound check
            elif point[0] < self.map_min[0] or point[0] > self.map_max[0] or \
                 point[1] < self.map_min[1] or point[1] > self.map_max[1] or \
                 point[2] < self.map_min[2] or point[2] > self.map_max[2]:
                continue    
            ## surface count
            if point[3] > self.surface_th[0] and point[3] < self.surface_th[1]: 
                self.surface_list.append(point)
                self.cnt_surface_voxel += 1
                if self.f_visualization: 
                    self.v_surface.append([point[0],point[1],point[2]])
            ## all covered volume
            self.covered_list.append(point)
            self.cnt_covered_voxel += 1
            if self.f_visualization: 
                self.v_covered.append([point[0],point[1],point[2]])
        
        self.covered_volume = self.cnt_covered_voxel * pow(self.voxel_size, 3)
        self.surface_volume = self.cnt_surface_voxel * pow(self.voxel_size, 3)
        self.per_covered_volume = 100 * self.covered_volume / self.gt_volume_m3
        self.per_surface_volume = 100 * self.surface_volume / self.gt_surface_m3 # self.gt_surface_m3
        self.per_covered_voxel = 100 * self.cnt_covered_voxel / self.gt_voxel
        self.per_surface_voxel = 100 * self.cnt_surface_voxel / self.gt_surface_voxel # self.gt_surface_voxel

        if self.f_visualization:
            tmp = pc2.create_cloud_xyz32(msg.header, self.v_covered)
            self.pub_debug.publish(tmp)

            x_min = self.map_min[0]; y_min = self.map_min[1]; z_min = self.map_min[2]
            x_max = self.map_max[0]; y_max = self.map_max[1]; z_max = self.map_max[2]
            marker = Marker()
            marker.pose.orientation.w = 1.0; marker.header.frame_id = "world"; marker.id = 0
            marker.type = marker.LINE_LIST; marker.action = marker.ADD
            p = Point(); p.x = x_min; p.y = y_min; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_min; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_min; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_max; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_min; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_min; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_min; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_max; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_min; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_min; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_max; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_max; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_max; p.z = z_min; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_max; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_min; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_min; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_min; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_max; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_max; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_min; p.y = y_max; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_max; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_min; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_max; p.z = z_max; marker.points.append(p)
            p = Point(); p.x = x_max; p.y = y_max; p.z = z_min; marker.points.append(p)
            marker.scale.x = 0.1; marker.color.a = 1.0; marker.color.r = 1.0
            self.pub_map_bound.publish(marker)
        return

    def eval_finish(self):
        self.result_file.close()
        rospy.loginfo("Evaluate shutdown: closing data files.")

if __name__ == '__main__':
    eval = evaluate()
    time.sleep(0.2)

    while 1:
        try:
            if eval.toggle_evaluate:
                if not eval.f_set_timer:
                    begin = time.time()
                    eval.f_set_timer = True
                if time.time() - begin > eval.log_interval:
                    eval.f_set_timer = False
                    if eval.f_verbose:
                        print("Time: {} min {} sec".format(round(time.time() - eval.eval_start_time,2)//60, round(time.time() - eval.eval_start_time,2)%60))
                        print("covered volume / surface: {}, {} [%]".format(round(eval.per_covered_volume,3), round(eval.per_surface_volume,3)))
                        print("voxelll volume / surface: {}, {} [%]".format(round(eval.per_covered_voxel,3), round(eval.per_surface_voxel,3)))
                        print("covered volume / surface: {}, {} [m^3]".format(round(eval.covered_volume,3), round(eval.surface_volume,3)))                
                    
                    eval.writer.writerow([round(time.time() - eval.eval_start_time,2), \
                                round(eval.per_covered_voxel,3), round(eval.per_surface_voxel,3), \
                                round(eval.covered_volume,3), round(eval.surface_volume,3)])
            else:
                continue
        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)