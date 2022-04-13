#!/usr/bin/env python3

from glob import glob
import queue
from termios import VEOL
from sympy import false, im
import rospy
import actionlib
import smach 
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench, Twist, Pose, Quaternion 
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from dutuuv_actions.msg import TurnAction, TurnGoal
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes

pi = 3.14159265

class LookAround(smach.State):
    def __init__(self, vel):
        smach.State.__init__(self, outcomes=['object recognized', 'no object recognized'])
        
        self.exit = False
        self.vel = vel
        self.t = 2 * pi / vel
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.bbox_sub = rospy.Subscriber('bbox', BoundingBoxes, callback=self.bbox_cb, queue_size=1)

    def execute(self, ud):
        rate = rospy.Rate(50)
        vel_msg = Twist()
        vel_msg.linear.z = self.vel
        start_t = rospy.Time.now()
        while rospy.Time.now() - start_t < rospy.Duration(secs=self.t) and not self.exit:
            self.vel_pub.publish(vel_msg)
            rate.sleep()
        # 机器人悬浮
        vel_msg.linear.z = 0.0
        self.vel_pub.publish(vel_msg)
        
        if self.exit: return 'object recognized' 
        else: return 'no object recognized'

    def bbox_cb(self, bboxes_msg: BoundingBoxes):
        if len(bboxes_msg.bounding_boxes) != 0:
            self.exit = True