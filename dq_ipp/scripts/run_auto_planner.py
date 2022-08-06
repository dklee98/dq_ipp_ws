#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import MultiDOFJointTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.srv import SetMode, CommandBool
from mavros_msgs.msg import State
from std_srvs.srv import SetBool
from collections import deque
# from tf2_msgs.msg import TFMessage
# import tf

import time
import sys
import signal

def signal_handler(signal, frame): # ctrl + c -> exit program
    print('You pressed Ctrl+C!')
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)


class run_auto_planner():
    def __init__(self):
        rospy.init_node('mav_active_3d_planning_node', anonymous=True)

        self.sub_uav_state = rospy.Subscriber('/mavros/state', State, self.cb_uav_state)
        self.sub_uav_pose = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.cb_uav_pose)
        self.sub_command_trajectory = rospy.Subscriber('/command/trajectory', MultiDOFJointTrajectory, self.cb_cmd_traj)

        self.pub_pose = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 2)
        self.pub_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=2)

        self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.offboarding = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.srv_run_planner = rospy.ServiceProxy('/planner_node/toggle_running', SetBool)

        self.rate = rospy.Rate(30)

        self.traj_deque = deque()

        self.state_in = False
        self.pose_in = False
        self.initialized = False


    def cb_uav_state(self, msg):
        self.state = msg
        self.state_in = True
        return

    def cb_uav_pose(self, msg):
        self.cur_local_pose = msg
        # if not self.pose_in:
        #     self.set_pose = msg
        self.set_pose = msg
        self.pose_in = True
        return


    def cb_cmd_traj(self, msg):
        array_size = len(msg.points)
        if array_size == 0:
            return

        for i in range(array_size):
            self.set_pose.header.stamp = rospy.Time.now()
            self.set_pose.pose.position.x = msg.points[i].transforms[0].translation.x
            self.set_pose.pose.position.y = msg.points[i].transforms[0].translation.y
            self.set_pose.pose.position.z = msg.points[i].transforms[0].translation.z
            self.set_pose.pose.orientation = msg.points[i].transforms[0].rotation

        print('--------------------------')
        print('traj in ')

        # for point in msg.points:
        #     point.transforms[0].translation # need vector3 from msg
        #     point.transforms[0].rotation    # need quaternion from msg
        #     # geometry msgs pointstamped? convert it
        #     # queue check whether reach the goal pose
        #     # pop and next goal set
        #     # 
        # self.traj_deque.append()
        
        

        return

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
                        if runner.cur_local_pose.pose.position.z > 1.0:
                            runner.initialized = True
                            runner.srv_run_planner(True)
                            print('run planner On')
                time.sleep(0.03)
                continue
            maintain_pose = runner.set_pose
            runner.pub_pose.publish(maintain_pose)

            runner.rate.sleep()

        except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt) :
            sys.exit(0)