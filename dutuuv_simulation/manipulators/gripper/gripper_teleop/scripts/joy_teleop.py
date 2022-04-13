#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy

class GripperJoyTeleop():
    def __init__(self) -> None:
        self.left_jaw_pub = rospy.Publisher('left_jaw_publish_topic', Float64, queue_size=1)
        self.right_jaw_pub = rospy.Publisher('right_jaw_publish_topic', Float64, queue_size=1)
        # self.joint_states_sub = rospy.Subscriber('/guppy/joint_states', JointState, self.joint_states_cb, queue_size=1)
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_cb, queue_size=1)
        self.left_jaw_position, self.right_jaw_position = 0.0, 0.0
        
        self.deadman_button = rospy.get_param('~deadman_button', 4)
        self.axis_subscript = rospy.get_param('~axis_subscript', 1)
        
        # 爪子的活动范围
        self.joint_limit_upper = rospy.get_param('~joint_limit_upper', 1.0)
        self.joint_limit_lower = rospy.get_param('~joint_limit_lower', -1.0)

    # def joint_states_cb(self, msg: JointState):
    #     self.left_jaw_position, self.right_jaw_position = msg.position[0], msg.position[1]
    
    def joy_cb(self, msg: Joy):
        if msg.buttons[self.deadman_button] and msg.axes[self.axis_subscript] : 
            gripper_scale = msg.axes[self.axis_subscript]
            
            left_jaw_msg, right_jaw_msg = Float64(), Float64()
            left_jaw_msg.data, right_jaw_msg.data = gripper_scale * self.joint_limit_upper, -gripper_scale * self.joint_limit_upper
             
            self.left_jaw_pub.publish(left_jaw_msg)
            self.right_jaw_pub.publish(right_jaw_msg)


if __name__ == '__main__':
    rospy.init_node('gripper_joy_teleop')
    _ = GripperJoyTeleop()
    rospy.spin()
    