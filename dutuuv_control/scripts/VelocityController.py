#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Twist, Vector3Stamped, Accel
from sensor_msgs.msg import Imu
from rospy.exceptions import ROSException
# from dutuuv_msgs.msg import AngularVelocityCommand, TorqueCommand
from dutuuv_control.PidController import PidController

import numpy
from dynamic_reconfigure.server import Server
# from dutuuv_control.config import ControllerPidConfig

class VelocityController():
    def __init__(self) -> None:

        try:

            self.linear_x_kp = rospy.get_param('~velocity_control/linear_x_kp') 
            self.linear_x_ki = rospy.get_param('~velocity_control/linear_x_ki')
            self.linear_x_kd = rospy.get_param('~velocity_control/linear_x_kd')

            self.linear_y_kp = rospy.get_param('~velocity_control/linear_y_kp') 
            self.linear_y_ki = rospy.get_param('~velocity_control/linear_y_ki')
            self.linear_y_kd = rospy.get_param('~velocity_control/linear_y_kd')

            self.linear_z_kp = rospy.get_param('~velocity_control/linear_z_kp')         
            self.linear_z_ki = rospy.get_param('~velocity_control/linear_z_ki')    
            self.linear_z_kd = rospy.get_param('~velocity_control/linear_z_kd')

            self.angular_x_kp = rospy.get_param('~velocity_control/angular_x_kp') 
            self.angular_x_ki = rospy.get_param('~velocity_control/angular_x_ki')
            self.angular_x_kd = rospy.get_param('~velocity_control/angular_x_kd')

            self.angular_y_kp = rospy.get_param('~velocity_control/angular_y_kp') 
            self.angular_y_ki = rospy.get_param('~velocity_control/angular_y_ki')
            self.angular_y_kd = rospy.get_param('~velocity_control/angular_y_kd')

            self.angular_z_kp = rospy.get_param('~velocity_control/angular_z_kp')         
            self.angular_z_ki = rospy.get_param('~velocity_control/angular_z_ki')    
            self.angular_z_kd = rospy.get_param('~velocity_control/angular_z_kd')
        except KeyError:
            rospy.logfatal('Angular Velocity Controller PID parameters KeyError')
            exit(0)

        # 订阅odom/imu
        self.odom_sub = rospy.Subscriber('odom', Odometry, callback=self.odom_cb, queue_size=1) 
        # self.imu_sub = rospy.Subscriber('imu_topic', Imu, callback=self.imu_cb, queue_size=1) 
        # 订阅cmd_vel
        self.vel_sub = rospy.Subscriber('cmd_vel', Twist, callback=self.vel_cb, queue_size=1)
        # 发布cmd_accel
        self.accel_pub = rospy.Publisher('cmd_accel', Accel, queue_size=1)

        self.linear_z_controller = PidController(self.linear_x_kp, self.linear_x_ki, self.linear_x_kd, 20)
        self.angular_x_controller = PidController(self.angular_x_kp, self.angular_x_ki, self.angular_x_kd, 1) 
        self.angular_y_controller = PidController(self.angular_y_kp, self.angular_y_ki, self.angular_y_kd, 1) 
        self.angular_z_controller = PidController(self.angular_z_kp, self.angular_z_ki, self.angular_z_kd, 1) 

        self.angular_setpoint = numpy.zeros(3)
        self.linear_setpoint = numpy.zeros(3)

        # self.dc_server = Server(WrenchControllerPidConfig, self.dr_callback)


    def odom_cb(self, odom_msg: Odometry):
       
        linear_z = odom_msg.twist.twist.linear.z
        linear_feedback = numpy.array([0.0, 0.0, linear_z])
        linear_error = self.linear_setpoint - linear_feedback 

        angular_x = odom_msg.twist.twist.angular.x
        angular_y = odom_msg.twist.twist.angular.y
        angular_z = odom_msg.twist.twist.angular.z
        angular_feedback = numpy.array([angular_x, angular_y, angular_z]) 
        
        angualr_error = self.angular_setpoint - angular_feedback


        t = odom_msg.header.stamp.to_sec()
        accel_linear_z = self.linear_z_controller.update(linear_error[2], t)
        accel_angular_x = self.angular_x_controller.update(angualr_error[0], t)
        accel_angular_y = self.angular_y_controller.update(angualr_error[1], t)
        accel_angular_z = self.angular_z_controller.update(angualr_error[2], t)

        accel_msg = Accel()
        accel_msg.linear.z = accel_linear_z
        accel_msg.angular.x = accel_angular_x
        accel_msg.angular.y = accel_angular_y
        accel_msg.angular.z = accel_angular_z
        
        self.accel_pub.publish(accel_msg)


    # 设置pid控制器输入
    def vel_cb(self, vel_msg: Twist):
        self.linear_setpoint[2] = vel_msg.linear.z
        self.angular_setpoint[0] = vel_msg.angular.x        
        self.angular_setpoint[1] = vel_msg.angular.y        
        self.angular_setpoint[2] = vel_msg.angular.z        


    def dr_callback(self, config, level):
        rospy.loginfo("""Reconfiugre Request: {angular_z_kp}""".format(**config))
        self.angular_y_pid['kp'] = config['angular_z_kp']
        self.angular_y_pid['ki'] = config['angular_z_ki']
        # print(self.angular_y_pid['kp'])
        self.yaw_controller.reset(self.angular_y_pid['kp'], self.angular_y_pid['ki'], 0.0)
        return config



if __name__ =='__main__':
    rospy.init_node('velocity_controller')
    _ = VelocityController()
    rospy.spin()