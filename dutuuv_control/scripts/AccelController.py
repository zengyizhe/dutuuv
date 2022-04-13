#!/usr/bin/env python3

import numpy
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Twist, Vector3Stamped, Accel
from sensor_msgs.msg import Imu
from rospy.exceptions import ROSException
from dutuuv_msgs.msg import AngularVelocityCommand 
from dutuuv_control.PidController import PidController


from dynamic_reconfigure.server import Server
# from dutuuv_control.cfg import WrenchControllerPidConfig

class AccelController():
    def __init__(self) -> None:

        try:
            self.mass = rospy.get_param('~mass') 
            self.inertia_params = rospy.get_param('~inertia')

        except KeyError:
            rospy.logfatal('Angular Velocity Controller PID parameters KeyError')
            exit(0)
        
        self.inertia_matrix = numpy.array([[self.inertia_params['ixx'], self.inertia_params['ixy'], self.inertia_params['ixz']], \
            [self.inertia_params['ixy'], self.inertia_params['iyy'], self.inertia_params['iyz']], \
            [self.inertia_params['ixz'], self.inertia_params['iyz'], self.inertia_params['izz']]])
        self.mass_inertia_matrix = numpy.vstack((
          numpy.hstack((self.mass*numpy.identity(3), numpy.zeros((3, 3)))),
          numpy.hstack((numpy.zeros((3, 3)), self.inertia_matrix))))


        self.effort_msg = Wrench()
        self.cmd_accel_sub = rospy.Subscriber('cmd_accel', Accel, callback=self.accel_cb, queue_size=1)
        self.cmd_force_sub = rospy.Subscriber('cmd_force', Wrench, callback=self.force_cb, queue_size=1) 
        # 发布 6DOF force & torque        
        self.effort_pub = rospy.Publisher('cmd_effort', Wrench, queue_size=1)

        # self.dc_server = Server(WrenchControllerPidConfig, self.dr_callback)

    # 加速度指令回调函数
    def accel_cb(self, accel_msg: Accel):
        
        linear = numpy.array([accel_msg.linear.x, accel_msg.linear.y, accel_msg.linear.z])
        angular = numpy.array([accel_msg.angular.x, accel_msg.angular.y, accel_msg.angular.z]) 
        accel = numpy.hstack([linear, angular])
        wrh = numpy.dot(self.mass_inertia_matrix, accel) 
        # wrench_msg = Wrench()
        # wrench_msg.force.x = wrh[0]
        # wrench_msg.force.y = wrh[1]
        self.effort_msg.force.z = wrh[2]
        self.effort_msg.torque.x = wrh[3]
        self.effort_msg.torque.y = wrh[4]
        self.effort_msg.torque.z = wrh[5]
        
        # self.effort_pub.publish(wrench_msg)

    # 力&力矩指令回调函数
    def force_cb(self, force_msg: Wrench):

        x = force_msg.force.x
        y = force_msg.force.y
        self.effort_msg.force.x = x
        self.effort_msg.force.y = y

    def spin(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.effort_pub.publish(self.effort_msg)
            rate.sleep()
            

    # def dr_callback(self, config, level):
    #     rospy.loginfo("""Reconfiugre Request: {angular_z_kp}""".format(**config))
    #     self.angular_y_pid['kp'] = config['angular_z_kp']
    #     self.angular_y_pid['ki'] = config['angular_z_ki']
    #     self.yaw_controller.reset(self.angular_y_pid['kp'], self.angular_y_pid['ki'], 0.0)
    #     return config


if __name__ =='__main__':

    rospy.init_node('accel_controller')
    accel_controller = AccelController()
    accel_controller.spin()
    rospy.spin()