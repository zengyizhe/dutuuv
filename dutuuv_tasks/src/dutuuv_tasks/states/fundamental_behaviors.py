#!/usr/bin/env python3

from glob import glob
import queue
from termios import VEOL
from sympy import im
import rospy
import actionlib
import smach 
from std_msgs.msg import Float64
from geometry_msgs.msg import Wrench, Twist, Pose, Quaternion 
from nav_msgs.msg import Odometry
# from dutuuv_control.msg import DepthControlAction, DepthControlGoal, DepthControlFeedback
from tf.transformations import quaternion_from_euler
from dutuuv_actions.msg import TurnAction, TurnGoal

pi = 3.14159265

class Up(smach.State):
    def __init__(self, vel, t):
        smach.State.__init__(self, outcomes=['success'])
        
        self.vel = abs(vel)
        self.t = t
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def execute(self, ud):
        rate = rospy.Rate(50)
        vel_msg = Twist()
        vel_msg.linear.z = self.vel
        start_t = rospy.Time.now()
        while rospy.Time.now() - start_t < rospy.Duration(secs=self.t):
            self.vel_pub.publish(vel_msg)
            rate.sleep()
        # 机器人悬浮
        vel_msg.linear.z = 0.0
        self.vel_pub.publish(vel_msg)
        return 'success'

class Down(smach.State):
    def __init__(self, vel, t):
        smach.State.__init__(self, outcomes=['success'])
        
        self.vel = -abs(vel)
        self.t = t
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def execute(self, ud):
        rate = rospy.Rate(50)
        vel_msg = Twist()
        vel_msg.linear.z = self.vel
        start_t = rospy.Time.now()
        while rospy.Time.now() - start_t < rospy.Duration(secs=self.t):
            self.vel_pub.publish(vel_msg)
            rate.sleep()
        # 机器人悬浮
        vel_msg.linear.z = 0.0
        self.vel_pub.publish(vel_msg)
        return 'success'


class Forward(smach.State):
    def __init__(self, thrust, t):
        smach.State.__init__(self, outcomes=['success'])
        
        self.thrust = abs(thrust)
        self.t = t
        self.thrust_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


    def execute(self, ud):
        init_time = rospy.Time.now()
        rate = rospy.Rate(50)
        thrust_msg = Twist()
        thrust_msg.linear.x = self.thrust
        
        while rospy.Time.now() - init_time < rospy.Duration(secs=self.t):
            self.thrust_pub.publish(thrust_msg)
            rate.sleep()
        # 机器人悬浮
        thrust_msg.linear.x = 0.0
        self.thrust_pub.publish(thrust_msg)
        
        return 'success'; 

class Backward(smach.State):
    def __init__(self, thrust, t):
        smach.State.__init__(self, outcomes=['success'])
        
        self.thrust = -abs(thrust)
        self.t = t
        self.thrust_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


    def execute(self, ud):
        init_time = rospy.Time.now()
        rate = rospy.Rate(50)
        thrust_msg = Twist()
        thrust_msg.linear.x = self.thrust
        
        while rospy.Time.now() - init_time < rospy.Duration(secs=self.t):
            self.thrust_pub.publish(thrust_msg)
            rate.sleep()
        # 机器人悬浮
        thrust_msg.linear.x = 0.0
        self.thrust_pub.publish(thrust_msg)
        
        return 'success'; 



class TurnLeft(smach.State):
    def __init__(self, vel:float, dura: int):
        smach.State.__init__(self, outcomes=['success'])
        self.vel = abs(vel)
        self.dura = dura
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    def execute(self, ud):
        ns = 'turn'
        client = actionlib.SimpleActionClient(ns, TurnAction)
        client.wait_for_server()
        goal = TurnGoal()
        goal.goal.vel = self.vel
        goal.goal.dura = self.dura
        client.send_goal(goal)
        client.wait_for_result()

        return 'success'

class TurnRight(smach.State):
    def __init__(self, vel:float, dura: int):
        smach.State.__init__(self, outcomes=['success'])
        self.vel = -abs(vel) # zhengadwwwwwwwdawdwaddawdawdadawdwadwwdwd
        self.dura = dura
        self.pub = rospy.Publisher('turn_topic', Wrench)

    def execute(self, ud):
        ns = 'turn'
        client = actionlib.SimpleActionClient(ns, TurnAction)
        client.wait_for_server()
        goal = TurnGoal()
        goal.goal.vel = self.vel
        goal.goal.dura = self.dura
        client.send_goal(goal)
        client.wait_for_result()

        return 'success'



class Stop(smach.State):
    def __init__(self, t: float):
        smach.State.__init__(self, outcomes=['success'])
        
        self.t = t
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        

    def execute(self, ud):
        rate = rospy.Rate(50)
        vel_msg = Twist()
         
        end_time = rospy.Time.now() + rospy.Duration(secs=self.t)        
        while rospy.Time().now() < end_time:
            self.vel_pub.publish(vel_msg)
            rate.sleep()
        
        return 'success'; 


# class LookAround(smach.State):
#     def __init__(self):
#         super().__init__(outcomes=['no_object_detected', 'object_detected'])
        
#         self.bbox_sub = rospy.Subscriber('bbox_topic', BoundingBoxes, callback=self.bbox_cb, queue_size=3)
#         self.odom_sub = rospy.Subscriber('odom', Odometry, callback=self.odom_cb, queue_size=1)
#         self.attitude_pub = rospy.Publisher('attitude_controller', Quaternion, queue_size=1)

#     def execute(self, ud):
#         global pi
#         quat = quaternion_from_euler(0, 0, 2 * pi)
#         msg = Quaternion()
#         msg.x = quat[0] 
#         msg.y = quat[1] 
#         msg.z = quat[2] 
#         msg.w = quat[3] 
#         self.attitude_pub.publish(msg)

#     def bbox_cb(self, msg: BoundingBoxes):
        


#         return 

#     def odom_cb(self, msg):
#         pass
