#!/usr/bin/python3
import sensor_msgs.msg
import rospy
from std_msgs.msg import Int16


def counter_callback(msg):
    print('getit')
    rospy.loginfo(msg.data)

rospy.init_node('Subscriber')
rospy.loginfo('aaa')
rospy.Subscriber('counter', Int16, callback=counter_callback, queue_size=1)

rospy.spin()

