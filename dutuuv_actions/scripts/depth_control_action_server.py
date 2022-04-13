#!/usr/bin/env python3

import rospy
import actionlib

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Joy

from dutuuv_control.controllers.pid_controller import PidController 
from dutuuv_control.msg import DepthControlAction, DepthControlGoal, DepthControlFeedback 
from dutuuv_msgs.msg import DepthCommand, Depth, DepthData
import time

class DepthControlActionServer():

    def __init__(self) -> None:
        
        self.current_depth = self.current_depth_vel = None

        self.odom_sub = rospy.Subscriber('odom', Odometry, callback=self.odom_cb, queue_size=1)
        self.vel_pub = rospy.Publisher('command/depth', Float64, queue_size=1)
        
        while self.current_depth is None and self.current_depth_vel is None:
            time.sleep(1)

        self._action_name = 'depth_control'
        self._feedback = DepthControlFeedback()
        self._as = actionlib.SimpleActionServer(self._action_name, DepthControlAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        # self.watchdog = None

    def depth_data_cb(self, msg:DepthData):
        self.current_depth = msg.depth
        self.current_depth_vel = msg.velocity


    def execute_cb(self, goal: DepthControlGoal):

        depth_cmd = None
        if goal.depth_control_action_goal.depth_keeping is True:
            depth_cmd = Float64()
            depth_cmd.data = self.current_depth
        else:
            depth_cmd = Float64()
            depth_cmd.data = goal.depth_control_action_goal.depth_command

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():

            self.depth_cmd_pub.publish(depth_cmd)

            # self._as.set_aborted(text="Tracking timeout")

            if self._as.is_preempt_requested():
                rospy.loginfo('{} Preempted'.format(self._action_name))


                self._as.set_preempted(text='Object Tracking controller preempted')
                return


            self._feedback.depth_control_action_feedback.current_depth = self.current_depth
            self._feedback.depth_control_action_feedback.goal_depth = goal.depth_control_action_goal.depth_command
            
            depth_error = goal.depth_control_action_goal.depth_command - self.current_depth
            self._feedback.depth_control_action_feedback.depth_error = depth_error
            self._as.publish_feedback(self._feedback)

            rate.sleep()

            

            
if __name__ =="__main__":
    rospy.init_node('depth_control_action_server')
    _ = DepthControlActionServer()
    rospy.spin()
