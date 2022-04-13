#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench, Twist, Vector3Stamped
from sensor_msgs.msg import Imu
from rospy.exceptions import ROSException
from dutuuv_msgs.msg import AngularVelocityCommand, TorqueCommand, Depth
from dutuuv_control.controllers.pid_controller import PidController
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from dynamic_reconfigure.server import Server
from dutuuv_control.config import BalanceControllerPidConfig

class BalanceController():
    def __init__(self) -> None:
        try:
            # 读取翻滚角pid控制器参数
            self.roll_p = rospy.get_param('~roll_p') 
            self.roll_i = rospy.get_param('~roll_i')
            self.roll_d = rospy.get_param('~roll_d')
            self.roll_i_clamp_max = rospy.get_param('~roll_i_clamp_max')
            self.roll_i_clamp_min = rospy.get_param('~roll_i_clamp_min')

            # 读取俯仰角pid控制器参数
            self.pitch_p = rospy.get_param('~pitch_p') 
            self.pitch_i = rospy.get_param('~pitch_i')
            self.pitch_d = rospy.get_param('~pitch_d')
            self.pitch_i_clamp_max = rospy.get_param('~pitch_i_clamp_max')
            self.pitch_i_clamp_min = rospy.get_param('~pitch_i_clamp_min')
        except KeyError:
            rospy.logfatal('Balance Controller PID parameters KeyError')
            exit(1)
        
        self.roll, self.pitch = 0.0, 0.0
        # 订阅惯性测量单元（imu）数据
        self.imu_sub = rospy.Subscriber('imu/data', Imu, callback=self.imu_cb, queue_size=1) 
        # 发布力矩指令
        self.torque_x_pub = rospy.Publisher('command/torque_x', Wrench, queue_size=1)
        self.torque_y_pub = rospy.Publisher('command/torque_y', Wrench, queue_size=1)
        
        self.roll_controller = PidController(self.roll_p, self.roll_i, self.roll_d, self.roll_i_clamp_max, self.roll_i_clamp_min) 
        self.pitch_controller = PidController(self.pitch_p, self.pitch_i, self.pitch_d, self.pitch_i_clamp_max, self.pitch_i_clamp_min) 
        
        self.dc_server = Server(WrenchControllerPidConfig, self.dr_callback)

    # 处理惯性测量单元（imu）数据
    def imu_cb(self, msg: Imu):
        quat = msg.orientation
        # 四元数 -> 欧拉角
        euler = euler_from_quaternion(quat)
        roll_feedback, pitch_feedback, _ = euler
        roll_setpoint, pitch_setpoint = 0.0, 0.0
        # 计算控制指令
        torque_x = self.roll_controller.update()
        torque_y = self.pitch_controller.update()
        # 发布控制指令
        self.torque_x_pub.publish(torque_x)
        self.torque_y_pub.publish(torque_y)


    def dr_callback(self, config, level):
        rospy.loginfo("""Reconfiugre Request: {angular_z_kp}""".format(**config))
        self.angular_y_pid['kp'] = config['angular_z_kp']
        self.angular_y_pid['ki'] = config['angular_z_ki']
        # print(self.angular_y_pid['kp'])
        self.yaw_controller.reset(self.angular_y_pid['kp'], self.angular_y_pid['ki'], 0.0)
        return config

if __name__ =='__main__':
    rospy.init_node('angular_vel_controller')
    controller = BalanceController()
    rospy.spin()








        
    

