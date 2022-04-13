#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Wrench

class JoyTeleop():
    def __init__(self) -> None:
        try:
            self.force_linear_x_max = rospy.get_param('~force_linear_x_max')
            self.force_linear_y_max = rospy.get_param('~force_linear_y_max')
            self.vel_linear_z_max = rospy.get_param('~vel_linear_z_max')
            self.vel_angular_z_max = rospy.get_param('~vel_angular_z_max')
        except KeyError:
            rospy.logerr('JoyTeleop limits parameters error')
            exit(1)            
        
        try:
            self.linear_x_axis = rospy.get_param('~linear_x_axis')
            self.linear_y_axis = rospy.get_param('~linear_y_axis')
            self.linear_z_axis = rospy.get_param('~linear_z_axis')
            self.angular_z_axis = rospy.get_param('~angular_z_axis')
            self.deadman_button = rospy.get_param('~deadman_button')
        except KeyError:
            rospy.logerr('JoyTeleop joy mapping parameters Error')
            exit(1)   

        
        # 订阅手柄输入
        self.joy_sub = rospy.Subscriber('joy', Joy, queue_size=1, callback=self.joy_cb)
        # 发布速度指令
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # 发布x/y方向推力指令
        self.force_pub = rospy.Publisher('cmd_force', Wrench, queue_size=1)


    def joy_cb(self, msg: Joy) -> None:
        # 判断deadman键按下与否
        if msg.buttons[self.deadman_button] == 1:
            
            vel_msg = Twist()
            force_msg = Wrench()
            # 前进
            vel_msg.linear.x = self.vel_linear_z_max * msg.axes[self.linear_x_axis]
            # 左移/右移
            vel_msg.linear.y = self.vel_linear_z_max * msg.axes[self.linear_y_axis]
            # 上浮/下潜
            vel_msg.linear.z = self.vel_linear_z_max * msg.axes[self.linear_z_axis]
            # 左转/右转
            vel_msg.angular.z = self.vel_angular_z_max * msg.axes[self.angular_z_axis]

            # self.force_pub.publish(force_msg)
            self.vel_pub.publish(vel_msg)

        else:
            vel_msg = Twist()
            force_msg = Wrench()

            # 命令机器人悬浮
            self.vel_pub.publish(vel_msg)
            # self.force_pub.publish(force_msg)

        

if __name__ == '__main__':
    
    rospy.init_node('joy_velocity_teleop')
    _ = JoyTeleop()
    rospy.spin()