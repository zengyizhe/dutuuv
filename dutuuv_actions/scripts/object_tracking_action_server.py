#!/usr/bin/env python3

from threading import current_thread

import rospy
import actionlib

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Twist
from sensor_msgs.msg import Imu, Joy
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes
from rospy.exceptions import ROSException

from dynamic_reconfigure.server import Server

from dutuuv_control.PidController import PidController 
from guppy_control.cfg import ObjectTrackingConfig
from guppy_control.msg import ObjectTrackingAction, ObjectTrackingGoal, ObjectTrackingFeedback, ObjectTrackingResult 

import time

class ObjectTrackingActionServer():

    def __init__(self) -> None:
        
        self._action_name = 'object_tracking'
        self._feedback = ObjectTrackingFeedback()
        self._result = ObjectTrackingResult()
        self._as = actionlib.SimpleActionServer(self._action_name, ObjectTrackingAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

        self.object_name = None
        self.object_area_ratio = None

        # Subscribe odometry topic
        # self.odom_sub = rospy.Subscriber('odom/raw', Odometry, callback=)
        # Publish wrench command 
        self.wrench_pub = rospy.Publisher('command/wrench', Wrench, queue_size=1)
        

        self.img_width = 640
        self.img_height = 480
        self.img_center = {'x_center': self.img_width / 2, 'y_center': self.img_height / 2}
        self.img_area = self.img_width * self.img_height

        self.x_error = 0.0
        self.y_error = 0.0
        self.safe_area = {'xmin': self.img_width / 2 - 20, 'ymin': self.img_height / 2 - 20, \
                            'xmax': self.img_width / 2 + 20, 'ymax': self.img_height / 2 + 20}

        try:
            self.torque_z_kp = rospy.get_param('~torque_z_kp') 
            self.torque_z_ki = rospy.get_param('~torque_z_ki')
            self.torque_z_kd = rospy.get_param('~torque_z_kd')

            self.force_z_kp = rospy.get_param('~force_z_kp') 
            self.force_z_ki = rospy.get_param('~force_z_ki')
            self.force_z_kd = rospy.get_param('~force_z_kd')
            
            self.force_x_kp = rospy.get_param('~force_x_kp')         
            self.force_x_ki = rospy.get_param('~force_x_ki')    
            self.force_x_kd = rospy.get_param('~force_x_kd')
        except:
            rospy.logfatal('Pid Parameters KeyError')
            exit(0)


        self.orientation_controller = PidController(self.torque_z_kp, self.torque_z_ki, self.torque_z_kd)
        self.depth_controller = PidController(self.force_z_kp, self.force_z_ki, self.force_z_kd)
        self.forward_controller = PidController(self.force_x_kp, self.force_x_ki, self.force_x_kd)


        self.object_x = 0
        self.object_y = 0

        self.watchdog = None

        self.dc_server = Server(ObjectTrackingConfig, self.dr_callback)


    def execute_cb(self, goal: ObjectTrackingGoal):

        self.object_name = goal.goal.object_name

        if self.object_name == '':
            self.shutdown()
            return
        
        self.object_area_ratio = goal.goal.object_area_ratio
        
        self.last_tracking_time = time.time()
        self.bboxes_sub = rospy.Subscriber('yolov5/bboxes', BoundingBoxes, callback=self.bboxes_cb,queue_size=1)
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            current_time = time.time()
            if current_time - self.last_tracking_time > 30.0:
                rospy.loginfo('Tracking failure')

                self.bboxes_sub.unregister()
                self._as.set_aborted(text="Tracking timeout")
                return

            if self._as.is_preempt_requested():
                rospy.loginfo('{} Preempted'.format(self._action_name))

                self.bboxes_sub.unregister()
                # self._result = ObjectTrackingResult()
                # self._result.result = 'preempted'

                self._as.set_preempted(text='Object Tracking controller preempted')
                return

            rate.sleep()

            

        
    def bboxes_cb(self, bboxes_msg: BoundingBoxes):

            
            object_count = 0
            for bbox in bboxes_msg.bounding_boxes:

                if bbox.Class == self.object_name:
                    object_count += 1
                    # Multiple objects detected
                    if object_count > 1:
                        rospy.loginfo('Multiple objects detected')
                        return 
                    else:
                        self.object_x = (bbox.xmin + bbox.xmax) / 2
                        self.object_y = (bbox.ymin + bbox.ymax) / 2

                        bbox_area = (bbox.xmax - bbox.xmin) * (bbox.ymax - bbox.ymin)
                        self.bbox_area_ratio = bbox_area / self.img_area


            # No objects detected
            if object_count == 0: 
                rospy.loginfo('No objects detected')                
                return

            self.last_tracking_time = time.time()

            self._feedback.feedback.x_deviation = self.img_center['x_center'] - self.object_x
            self._feedback.feedback.y_deviation = self.img_center['y_center'] - self.object_y
            self._feedback.feedback.goal_area_ratio = self.object_area_ratio
            self._feedback.feedback.current_area_ratio = self.bbox_area_ratio

            self._as.publish_feedback(self._feedback)

            torque_z = self.orientation_controller.update(self.img_center['x_center'], self.object_x)
            force_z = self.depth_controller.update(self.img_center['y_center'], self.object_y)
            force_x = self.forward_controller.update(self.object_area_ratio, self.bbox_area_ratio)

            wrench_msg = Wrench()
            wrench_msg.force.z = force_z
            wrench_msg.torque.z = torque_z
            wrench_msg.force.x = force_x

            self.wrench_pub.publish(wrench_msg)
            
            
        
    def odom_cb(self, msg: Odometry):
        self.current_depth = msg.pose.pose.position.z

            
    def dr_callback(self, config, level):
        rospy.loginfo('Reconfigure Request')
        self.torque_z_kp = config['torque_z_kp']
        self.torque_z_ki = config['torque_z_ki']
        self.torque_z_kd = config['torque_z_kd']

        self.force_z_kp = config['force_z_kp']
        self.force_z_ki = config['force_z_ki']
        self.force_z_kd = config['force_z_kd']

        self.force_x_kp = config['force_x_kp']
        self.force_x_ki = config['force_x_ki']
        self.force_x_kd = config['force_x_kd']

        self.orientation_controller.reset(self.torque_z_kp, self.torque_z_ki, self.torque_z_kd)
        self.depth_controller.reset(self.force_z_kp, self.force_z_ki, self.force_z_kd)
        self.forward_controller.reset(self.force_x_kp, self.force_x_ki, self.force_x_kd)

        return config


    def shutdown(self):
        """
        Shutdown the robot
        """
        wrench_msg = Wrench()
        self.wrench_pub.publish(wrench_msg)

    

if __name__ =="__main__":

    rospy.init_node('object_tracking_action_server')
    _ = ObjectTrackingActionServer()
    rospy.spin()
