#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Twist, Vector3Stamped
from sensor_msgs.msg import Imu
from rospy.exceptions import ROSException
from dutuuv_msgs.msg import DepthCommand, DepthData, DepthForceCommand
from dutuuv_control.controllers.pid_controller import PidController
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from dutuuv_control.cfg import DepthVelocityControllerConfig

from time import monotonic, sleep
import math

class DepthController():

    def __init__(self) -> None:
        try:
            self.depth_vel_controller_kp = rospy.get_param('~depth_velocity_controller_kp') 
            self.depth_vel_controller_ki = rospy.get_param('~depth_velocity_controller_ki')
            self.depth_vel_controller_kd = rospy.get_param('~depth_velocity_controller_kd')
        except KeyError:
            rospy.logfatal('Depth Velocity Controller PID Parameters KeyError')
            exit(0)
        except ROSException:
            rospy.logfatal('Parameter Server ROSException')
            exit(0)
        
        self.current_depth_vel = None
        self.depth_data_sub = rospy.Subscriber('depth/data', DepthData, callback=self.depth_data_cb, queue_size=1)
        while self.current_depth_vel is None:
            sleep(1)
            rospy.logerr('depth data have not been published, keep waiting')
        self.depth_vel_controller = PidController(self.depth_vel_controller_kp, self.depth_vel_controller_ki, self.depth_vel_controller_kd) 
        self.depth_vel_cmd_sub = rospy.Subscriber('command/depth_velocity', Float64, callback=self.depth_vel_cmd_cb, queue_size=1)
        
        # self.imu_sub = rospy.Subscriber('imu/data', Imu, callback=self.imu_cb, queue_size=1) 
        
        # 发布线性力指令
        self.force_pub = rospy.Publisher('command/depth_force', Vector3Stamped, queue_size=1)

        self.dc_server = Server(DepthVelocityControllerConfig, self.dr_cb)

    def depth_data_cb(self, msg: DepthData):
        self.current_depth_vel = msg.velocity
        rospy.loginfo('current depth velocity: {}'.format(self.current_depth_vel))


    # def imu_cb(self, imu_msg: Imu):
    #     self.current_vel_x = imu_msg.angular_velocity.x
    #     self.current_vel_y = imu_msg.angular_velocity.y
    #     self.current_vel_z = imu_msg.angular_velocity.z

    def depth_vel_cmd_cb(self, msg: Float64):
        
            goal_depth_vel = msg.data 
            rospy.loginfo('goal depth velocity: {}'.format(goal_depth_vel))
                
            goal_force = self.depth_vel_controller.update(goal_depth_vel, self.current_depth_vel)
            rospy.loginfo('goal depth force: {}'.format(goal_force))

            # 不忽略机器人姿态
            depth_force_cmd = Vector3Stamped()
            depth_force_cmd.vector.x = 0.0
            depth_force_cmd.vector.y = 0.0
            depth_force_cmd.vector.z = goal_force
            self.depth_force_pub.publish(depth_force_cmd)
            rospy.loginfo('command/depth_force: [{}, {}, {}]'.format(depth_force_cmd.vector.x, depth_force_cmd.vector.y, depth_force_cmd.vector.z))

            # 忽略机器人姿态
            depth_force_cmd = Vector3Stamped()
            depth_force_cmd.vector.x = 0.0
            depth_force_cmd.vector.y = 0.0
            depth_force_cmd.vector.z = goal_force
            self.depth_force_pub.publish(depth_force_cmd)
            rospy.loginfo('command/depth_force: [{}, {}, {}]'.format(depth_force_cmd.vector.x, depth_force_cmd.vector.y, depth_force_cmd.vector.z))


    def dr_cb(self, config, level):
        
        # rospy.loginfo("""Reconfiugre Request: {angular_z_kp}""".format(**config))
        self.depth_vel_controller_kp = config['depth_velocity_controller_kp']
        self.depth_vel_controller_ki = config['depth_velocity_controller_ki']
        self.depth_vel_controller_kd = config['depth_velocity_controller_kd']
        
        self.depth_vel_controller.reset(self.depth_vel_controller_kp , self.depth_vel_controller_ki , self.depth_vel_controller_kd)
        return config

if __name__ =='__main__':

    rospy.init_node('angular_vel_controller')
    _ = DepthController()
    rospy.spin()



        
    

