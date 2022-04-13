#!/usr/bin/env python3

from time import time
import rospy
import smach 
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes
from geometry_msgs.msg import Twist


class ObjectTrackingState(smach.State):
    def __init__(self, goal: ObjectTrackingGoal):
        smach.State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.img_width = 640
        self.img_height = 480

        self.goal = goal
        

    def execute(self, userdata):
        self.action_client.send_goal(self.goal, feedback_cb=self.feedback_cb)
        self.action_client.wait_for_result()
        
        result_state = self.action_client.get_state()
        
        # actionlib PREEMPTED=2
        if result_state == 2: 
            return 'preempted'
        
        # actionlib SUCCEEDED=3
        elif result_state == 3:
            return 'succeeded'
        
        # actionlib ABORTED=4
        elif result_state == 4:
            return 'aborted'



    def bboxes_cb(self, bboxes_msg: BoundingBoxes):

        bbox = bboxes_msg.bounding_boxes[0]
        self.object_x = (bbox.xmin + bbox.xmax) / 2
        self.object_y = (bbox.ymin + bbox.ymax) / 2

        bbox_area = (bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin)
        
        self.bbox_area_ratio = bbox_area / self.img_area

        self.last_tracking_time = time.time()

        pixel_err = (self.img_width / 2 - self.object_x)

        t = time()
        vel = self.orientation_controller.update(pixel_err, t)

        vel_msg = Twist()
        vel_msg.linear.x = 0.1
        vel_msg.angular.z = vel
        self.vel_pub.publish(vel_msg)