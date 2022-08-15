#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from std_srvs.srv import SetBool
from collections import deque
# from tf2_msgs.msg import TFMessage
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_inverse

import time
import sys
import signal
import math
import copy

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class run_auto_planner():
    def __init__(self):
        rospy.init_node('auto_planning_node', anonymous=True)

        self.sub_uav_state = rospy.Subscriber('/mavros/state', State, self.cb_uav_state)
        self.sub_uav_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.cb_uav_pose)
        # self.sub_command_trajectory = rospy.Subscriber('/command/trajectory', MultiDOFJointTrajectory, self.cb_cmd_traj)
        self.sub_command_point = rospy.Subscriber('/command/trajectory', PoseStamped, self.cb_cmd_traj)

        self.pub_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 2)
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=2)

        # visualization
        self.pub_target_pose = rospy.Publisher("/visualization/target_pose", Marker, queue_size=10)

        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.srv_run_planner = rospy.ServiceProxy('/planner_node/toggle_running', SetBool)

        self.rate = rospy.Rate(30)

        self.traj_deque = deque()

        self.state_in = False
        self.pose_in = False
        self.cmd_in = False
        self.initialized = False

        self.error = [0,0,0,0]


    def cb_uav_state(self, msg):
        self.state = msg
        self.state_in = True
        return

    def cb_uav_pose(self, msg):
        self.cur_local_pose = msg.pose
        self.cur_q = [self.cur_local_pose.orientation.x,
                    self.cur_local_pose.orientation.y,
                    self.cur_local_pose.orientation.z,
                    self.cur_local_pose.orientation.w]
        self.cur_yaw = euler_from_quaternion(self.cur_q)[2]
        if not self.pose_in:
            self.set_pose = msg
            print("nono")
        self.pose_in = True
        return


    def cb_cmd_traj(self, msg):
        self.cmd_in = True
        self.set_pose = msg
        self.target_pose = msg.pose
        self.target_q = [self.target_pose.orientation.x,
                        self.target_pose.orientation.y,
                        self.target_pose.orientation.z,
                        self.target_pose.orientation.w]
        self.target_yaw = euler_from_quaternion(self.target_q)[2]
        self.error[0] = self.target_pose.position.x - self.cur_local_pose.position.x
        self.error[1] = self.target_pose.position.y - self.cur_local_pose.position.y
        self.error[2] = self.target_pose.position.z - self.cur_local_pose.position.z
        
        # q = self.target_q * quaternion_inverse(self.cur_q)
        # self.error[3] = euler_from_quaternion(q)[2]
        self.error[3] = math.fmod(self.target_yaw - self.cur_yaw, 2*math.pi)

        vel_cmd = TwistStamped()
        vel_cmd.header.stamp = rospy.Time.now()
        linear_kp = 0.5
        linear_bound = 1.0
        vel_cmd.twist.linear.x = self.bound(self.error[0] * linear_kp, linear_bound)
        vel_cmd.twist.linear.y = self.bound(self.error[1] * linear_kp, linear_bound)
        vel_cmd.twist.linear.z = self.bound(self.error[2] * linear_kp, linear_bound)
        vel_cmd.twist.angular.z = self.bound(self.error[3] * 0.5, 0.8)
        self.pub_vel.publish(vel_cmd)

        print('--------------------------')
        print('traj in ')

        # visualization
        marker = Marker()
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.scale.x = 0.5
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose = self.target_pose
        self.pub_target_pose.publish(marker)
        return

    def bound(self, data, bound):
        return data if abs(data) < bound else bound if data > 0.0 else -bound

if __name__ == '__main__':
    runner = run_auto_planner()
    time.sleep(0.2)

    while 1:
        try:
            if runner.state_in and not runner.initialized:
                if not runner.state.armed:
                    runner.arming(True)
                    continue
                else:
                    take_off_input = TwistStamped()
                    take_off_input.header.stamp = rospy.Time.now()
                    take_off_input.twist.linear.z = 1.0
                    runner.pub_vel.publish(take_off_input)

                if runner.state.mode != "OFFBOARD":
                    runner.offboarding(base_mode = 0, custom_mode = "OFFBOARD")
                else:
                    if runner.pose_in:
                        if runner.cur_local_pose.position.z > 1.0:
                            runner.initialized = True
                            runner.srv_run_planner(True)
                            print('run planner On')
                            time.sleep(1.0)
                time.sleep(0.03)
                continue
            
            if not runner.cmd_in:
                maintain_pose = runner.set_pose
                runner.pub_pose.publish(maintain_pose)

            runner.rate.sleep()

        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)