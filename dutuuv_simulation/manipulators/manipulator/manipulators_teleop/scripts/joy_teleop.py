#! /usr/bin/env python3
from time import time
import rospy
from std_msgs.msg import Float64MultiArray, Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Joy, JointState

class my_manipulator_joy_teleop():
    def __init__(self) -> None:
        while True:
            try:
                msg = rospy.wait_for_message('/dlut_auv/joint_states', JointState, timeout=10)
                break
            except:
                rospy.loginfo('waiting for joint states!!!')
        

        self.shoulder_azimuth_index = msg.name.index('shoulder_azimuth')
        self.shoulder_elevation_index = msg.name.index('shoulder_elevation')
        self.first_elbow_index = msg.name.index('first_elbow')
        self.second_elbow_index = msg.name.index('second_elbow')
        self.upper_jaws_joint_index = msg.name.index('upper_jaws_joint')
        self.lower_jaws_joint_index = msg.name.index('lower_jaws_joint')

        self.current_shoulder_azimuth_position = 0
        self.current_shoulder_elevation_position = 0
        self.current_first_elbow_position = 0
        self.current_second_elbow_position = 0
        self.current_lower_jaws_joint_position = 0
        self.current_upper_jaws_joint_position = 0

        self.arm_cmd_publisher = rospy.Publisher('/dlut_auv/my_manipulator/arm_controller/command', JointTrajectory, queue_size=1)
        self.hand_cmd_publisher = rospy.Publisher('dlut_auv/my_manipulator/hand_controller/command', Float64MultiArray, queue_size=1)
        self.joy_subscriber = rospy.Subscriber('dlut_auv/joy', Joy, callback=self.joy_callback, queue_size=1)
        self.joint_states_subscriber = rospy.Subscriber('dlut_auv/joint_states', JointState ,callback=self.joint_states_callback, queue_size=1)


    def joint_states_callback(self, msg):
        self.current_shoulder_azimuth_position = msg.position[self.shoulder_azimuth_index]
        self.current_shoulder_elevation_position = msg.position[self.shoulder_elevation_index]
        self.current_first_elbow_position = msg.position[self.first_elbow_index]
        self.current_second_elbow_position = msg.position[self.second_elbow_index]
        
        self.current_lower_jaws_joint_position = msg.position[self.lower_jaws_joint_index]
        self.current_upper_jaws_joint_position = msg.position[self.upper_jaws_joint_index]


    def joy_callback(self, msg):
        if msg.buttons[5] == 1.0:
            if msg.buttons[2] == 1.0:

                cmd = Float64MultiArray()
                cmd.data = [self.current_upper_jaws_joint_position + 0.1, self.current_lower_jaws_joint_position - 0.1]
                self.hand_cmd_publisher.publish(cmd)
            elif msg.buttons[1] == 1.0:
                cmd = Float64MultiArray()
                cmd.data = [self.current_upper_jaws_joint_position - 0.1, self.current_lower_jaws_joint_position + 0.1]
                self.hand_cmd_publisher.publish(cmd)
            else:
                pass
            cmd = JointTrajectory()
            cmd.joint_names = ['shoulder_azimuth', 'shoulder_elevation', 'first_elbow', 'second_elbow']
            sub_cmd = JointTrajectoryPoint()
            sub_cmd.positions = [self.current_shoulder_azimuth_position + (0.5 * msg.axes[3]), \
                                    self.current_shoulder_elevation_position - (0.5 * msg.axes[4]), \
                                    self.current_first_elbow_position - (0.5 * msg.axes[1]), \
                                    self.current_second_elbow_position - (0.5 * msg.axes[7])]
            _ = Duration()
            _.data.secs = 1
            # _.data = 
            # cmd.header.stamp = rospy.Time.now()
            sub_cmd.time_from_start = _.data
            cmd.points.append(sub_cmd)
            self.arm_cmd_publisher.publish(cmd)

if __name__ == '__main__':
    rospy.init_node('my_manipulator_joy_teleop')
    _ = my_manipulator_joy_teleop()
    rospy.spin()