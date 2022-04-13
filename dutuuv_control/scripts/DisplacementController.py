#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Wrench, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from dutuuv_msgs.msg import DepthData
from std_msgs.msg import Float64
from dutuuv_control.Pidcontroller import Pidcontroller

from dynamic_reconfigure.server import Server
from dutuuv_control.config import DepthControllerConfig

import time
import math

class DepthController():
    def __init__(self) -> None:
        try:
            self.depth_controller_kp = rospy.get_param('~depth_controller_kp') 
            self.depth_controller_ki = rospy.get_param('~depth_controller_ki')
            self.depth_controller_kd = rospy.get_param('~depth_controller_kd')
        except KeyError:
            rospy.logfatal('未载入深度控制器pid参数')
            exit(1)
        
        self.depth_feedback = None
        self.depth_setpoint = 0.0
        self.depth_controller = PidController(self.depth_controller_kp, self.depth_controller_ki, self.depth_controller_kd) 
        
        # 订阅上浮/下潜指令
        self.depth_sub = rospy.Subscriber('~cmd_depth', Float64, callback=self.depth_cb, queue_size=1)
        # 订阅深度计数据
        self.odom_sub = rospy.Subscriber('odom', Odometry, callback=self.odom_cb, queue_size=1)

        # 发布上浮/下潜速度指令
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        
        while self.current_depth is None:
            time.sleep(0.5)

        # self.imu_sub = rospy.Subscriber('imu/data', Imu, callback=self.imu_cb, queue_size=1) 
        
        


        self.dc_server = Server(DepthControllerConfig, self.dr_cb)

    # 里程数据回调函数
    def odom_cb(self, odom_msg: Odometry):
        self.depth_feedback = odom_msg.pose.pose.position.z
        self.t = odom_msg.header.stamp.to_sec()

    def depth_cb(self, depth_msg: Float64):
        # 获取目标深度
        depth_setpoint = depth_msg.data
        err = self.depth_setpoint - self.depth_feedback
       
        # 更新目标上浮/下潜速度
        output = self.depth_controller.update(err, self.t)
        # 发布目标上浮/下潜速度
        vel_msg = Twist()
        vel_msg.linear.z = output
        self.vel_pub.publish(vel_msg)

    def spin(self):
        rate = rospy.Rate(50)
        while (not rospy.is_shutdown()) and self. 

    # def dr_cb(self, config, level):
    #     # rospy.loginfo("""Reconfiugre Request: {angular_z_kp}""".format(**config))
    #     self.depth_controller_kp = config['depth_controller_kp']
    #     self.depth_controller_ki = config['depth_controller_ki']
    #     self.depth_controller_kd = config['depth_controller_kd']
    #     self.depth_controller.reset(self.depth_controller_kp, self.depth_controller_ki, self.depth_controller_kd)
    #     return config




if __name__ =='__main__':

    rospy.init_node('depth_controller')
    _ = DepthController()
    rospy.spin()