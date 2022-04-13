#!/usr/bin/env python3

import rospy
import actionlib
import smach 
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench, Twist, Pose, Quaternion 
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from dutuuv_actions.msg import TurnAction, TurnGoal

pi = 3.14159265

class SitDown(smach.State):
    def __init__(self, vel):
        smach.State.__init__(self, outcomes=['success'])
        
        self.exit = False
        self.vel = vel
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('odom', Odometry, callback=self.odom_cb, queue_size=1)

    def execute(self, ud):
        rate = rospy.Rate(50)
        vel_msg = Twist()
        vel_msg.linear.z = self.vel
        # start_t = rospy.Time.now()
        while not self.exit:
            self.vel_pub.publish(vel_msg)
            rate.sleep()
        # 机器人悬浮
        vel_msg.linear.z = 0.0
        self.vel_pub.publish(vel_msg)
        
        return 'success'
        
    def odom_cb(self, odom_msg: Odometry):
        vel = odom_msg.twist.twist.linear.z
        if abs(vel) < 0.05:   
            self.exit = True