#!/usr/bin/env python3
from math import dist
import rospy
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, TransformStamped, PointStamped
from sensor_msgs.msg import Image, PointCloud2
from rospy.core import logerr, loginfo
# from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d 
import cv2 as cv 
import numpy as np
import struct
import tf
from visualization_msgs.msg import MarkerArray
import sys
from moveit_commander import move_group
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
import time

class object_tracker:
    def __init__(self):

        joint_state_topic = ['joint_states:=/dlut_auv/joint_states']
        moveit_commander.roscpp_initialize(joint_state_topic)
        moveit_commander.roscpp_initialize(sys.argv)
        # self.robot = moveit_commander.RobotCommander()


        hand_group_name = 'my_manipulator_hand'
        arm_group_name = 'my_manipulator_arm'
        self.move_group = moveit_commander.MoveGroupCommander(arm_group_name)
        self.hand_move_group = moveit_commander.MoveGroupCommander(hand_group_name)
        # robot = moveit_commander.RobotCommander()
        
        self.goal_position_tolerance = 0.02
        self.goal_orientation_tolerance = 0.1
        self.move_group.set_goal_orientation_tolerance(self.goal_orientation_tolerance)
        self.move_group.set_goal_position_tolerance(self.goal_position_tolerance)
        self.move_group.allow_replanning(True)
        
        # self.move_group.set_pose_reference_frame("base_link")
        self.move_group.set_named_target('home')
        self.move_group.go(True)
        self.hand_move_group.set_named_target('close')
        self.hand_move_group.go(True)
        
        # self.hand_move_group.set_named_target('open')

        self.cmd_vel_pub = rospy.Publisher('/dlut_auv/cmd_vel', Twist, queue_size=1) 
        self.bounding_boxes_sub = rospy.Subscriber('/darknet_ros_3d/bounding_boxes', BoundingBoxes3d, callback=self.tracking_control, queue_size=1)
        # self.pose_sub = rospy.Subscriber('/darknet_ros_3d/markers', MarkerArray, callback=self.tracking_control, queue_size=1)
        self.pub = rospy.Publisher('test', PointStamped, queue_size=1)
        # self.pub_ = rospy.Publisher('test', Pose, queue_size=1)
        
        # self.tfBuffer = tf.Buffer()
        self.listener = tf.TransformListener()      
        self.cmd_vel = Twist()  
        self.x_setpoint = 0.31
        self.y_setpoint = 0.0
        self.z_setpoint = -0.05
    
        self.k_p = rospy.get_param('~/pid/k_p', default=0.4)
        self.moveit_status()


    def moveit_status(self):
                # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # self.move_group.set_pose_reference_frame("shoulder_link")
        # planning_frame = self.move_group.get_planning_frame()
        # print("============ Planning frame: %s" % planning_frame)

        # # We can also print the name of the end-effector link for this group:
        # eef_link = self.move_group.get_end_effector_link()
        # print("============ End effector link: %s" % eef_link)

        # # We can get a list of all the groups in the robot:
        # group_names = self.robot.get_group_names()
        # print("============ Available Planning Groups:", self.robot.get_group_names())

        # print("============ Printing robot state")
        # print(self.robot.get_current_state())

        # current_pose = self.move_group.get_current_pose().pose
        # self.pub_.publish(current_pose)


    def tracking_control(self, msg):
        if msg.bounding_boxes != None:
        # if msg.markers != None:

            # tf2Broadcaster = tf2_ros.TransformBroadcaster()
            # tf2Stamp = TransformStamped()
            # tf2Stamp.header.stamp = rospy.Time.now()
            # tf2Stamp.header.frame_id = 'dlut_auv/camera_link_optical'
            # tf2Stamp.child_frame_id = 'dlut_auv/base_link'

            pointstamped = PointStamped()
            pointstamped_ = PointStamped()

            pointstamped.header.frame_id = 'dlut_auv/camera_link_optical'
            pointstamped.header.stamp = rospy.Time(0)
            pointstamped.point.x = (msg.bounding_boxes[0].xmin + msg.bounding_boxes[0].xmax) / 2
            pointstamped.point.y = (msg.bounding_boxes[0].ymin + msg.bounding_boxes[0].ymax) / 2 
            pointstamped.point.z = (msg.bounding_boxes[0].zmin + msg.bounding_boxes[0].zmax) / 2 
            # pointstamped.point.x = msg.markers[0].pose.position.x
            # pointstamped.point.y = msg.markers[0].pose.position.y
            # pointstamped.point.z = msg.markers[0].pose.position.z
            # self.pub_.publish(current_pose)
            # print(pointstamped.point.x, \
            # pointstamped.point.y, \
            # pointstamped.point.z)
            # try:
            pointstamped_ = self.listener.transformPoint('base_link', pointstamped)
            pointstamped__ = self.listener.transformPoint('dlut_auv/base_link', pointstamped)
            self.pub.publish(pointstamped__)

            # (trans, rot) = self.listener.lookupTransform('dlut_auv/base_link','shoulder_azimuth',rospy.Time(0))
            # print((trans,rot))


            error_x = pointstamped_.point.x - self.x_setpoint
            error_y = pointstamped_.point.y - self.y_setpoint
            error_z = pointstamped_.point.z - self.z_setpoint

            if error_x <= 0.03 and error_y <= 0.03 and error_z <= 0.01:


            
                pose_goal = Pose()
            
            
                print('open the jaw')
                self.hand_move_group.set_named_target('open')
                self.hand_move_group.go(True)
                self.hand_move_group.stop()
            
                # current_pose = move_group.get_current_pose().pose
                pose_goal.position.x = pointstamped__.point.x 

                pose_goal.position.y = pointstamped__.point.y

                pose_goal.position.z = pointstamped__.point.z + 0.05

                pose_goal.orientation.y = 0.2570806
                pose_goal.orientation.w = 0.96639 
                # pose_goal.orientation.y = 0.8509035

                # pose_goal.orientation.w= 0.525322
                # 0.3824995, 0, 0.9239557 ]


                # pose_goal.orientation.w= 1.00000



                self.move_group.set_pose_target(pose_goal)
                print('starting planing')
                self.move_group.go(True)
                self.move_group.stop()


                # print('open the jaw')
                # self.hand_move_group.set_named_target('open')
                # self.hand_move_group.go(True)
                # self.hand_move_group.stop()


                
                # current_pose = self.move_group.get_current_pose().pose
                # pose_goal.position.x = current_pose.position.x
                # pose_goal.position.y= current_pose.position.y
                # pose_goal.position.z= current_pose.position.z - 0.05
                # pose_goal.orientation.y = 0.3824995 
                # pose_goal.orientation.w = 0.9239557 
                # self.move_group.set_pose_target(pose_goal)
                # print('go down')
                # self.move_group.go(True)
                # self.move_group.stop()


                print('close the jaw')
                self.hand_move_group.set_named_target('close')
                self.hand_move_group.go(True)
                self.hand_move_group.stop()
                # time.sleep(8)

                # self.move_group.set_named_target('home')
                # self.move_group.go(True)

                return

            self.cmd_vel.linear.x = self.k_p * error_x
            self.cmd_vel.linear.y = self.k_p * error_y 
            self.cmd_vel.linear.z = self.k_p * error_z 
            self.cmd_vel_pub.publish(self.cmd_vel)



            # except:
            #     loginfo('transform error')


                
                # index = (400 * msg.row_step) + (0 * msg.point_step) 
                # point = struct.unpack_from('fff', msg.data, offset=index)
                # print(point) 
                # goal_dist = 0.9
                # dist_error = dist_to_object - goal_dist
                # if dist_error >= 0.05:
                # self.cmd_vel.linear.y = -self.k_p * error_x
                # self.cmd_vel.linear.x = self.k_p * dist_error
                # self.cmd_vel.linear.z = -self.k_p * dist_error
                # self.cmd_vel.linear.y = -self.k_p * (point - 320)
                # self.cmd_vel_pub.publish(self.cmd_vel)
        else:
            print('zhezhule!!!!!!!!')

    # def tracking_control(self, msg):
    #     if(len(msg.bounding_boxes) == 0):
    #         loginfo('No object detected!!!')
    #         return
    #     elif(len(msg.bounding_boxes) != 1):
    #         loginfo('Multiple objects detected!!!')
    #         return
    #     else:
    #         self.object_x = (msg.bounding_boxes[0].xmax + msg.bounding_boxes[0].xmin) // 2
    #         self.object_y = (msg.bounding_boxes[0].ymax + msg.bounding_boxes[0].ymin) // 2

            # print(self.object_x, self.object_y)
            # object_center = (object_x, object_y)
            # error_x = object_x - 320
            # error_y = object_y - 320
            # self.cmd_vel.linear.y = -self.k_p * error_x
            # self.cmd_vel.linear.x = -self.k_p * error_y
            # self.cmd_vel_pub.publish(self.cmd_vel)

if __name__ == '__main__':
    rospy.init_node('object_tracker')
    _ = object_tracker()
    rospy.spin()







