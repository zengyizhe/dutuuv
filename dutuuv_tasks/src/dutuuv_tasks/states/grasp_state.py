#!/usr/bin/env python3
import rospy
import smach 
from std_msgs.msg import Float64
import time
pi = 3.14159265

class Grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'])
        
        self.left_jaw_pub = rospy.Publisher('/tank/left_jaw_controller/command', Float64, queue_size=1)
        self.right_jaw_pub = rospy.Publisher('/tank/right_jaw_controller/command', Float64, queue_size=1)

        left_jaw_msg, right_jaw_msg = Float64(), Float64()
        left_jaw_msg.data, right_jaw_msg.data = 1.0, -1.0
             
        self.left_jaw_pub.publish(left_jaw_msg)
        self.right_jaw_pub.publish(right_jaw_msg)


    def execute(self, ud):
        left_jaw_msg, right_jaw_msg = Float64(), Float64()
        left_jaw_msg.data, right_jaw_msg.data = -1.0, 1.0
        rate = rospy.Rate(50)
        start_t = rospy.Time.now()
        while rospy.Time.now() - start_t < rospy.Duration(secs=5.0):
            self.left_jaw_pub.publish(left_jaw_msg)
            self.right_jaw_pub.publish(right_jaw_msg)
        # 机器人悬浮
        # vel_msg.linear.z = 0.0
        # self.vel_pub.publish(vel_msg)
        
        return 'success'