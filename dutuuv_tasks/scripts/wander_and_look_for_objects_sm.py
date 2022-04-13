#!/usr/bin/env python3
import rospy
import actionlib
import smach 
from dutuuv_tasks.states.fundamental_behaviors import Forward, Backward, Down, Up, Stop 
from dutuuv_tasks.states.lookaround_state import LookAround 
from yolov5_pytorch_ros.msg import BoundingBox, BoundingBoxes

wander = smach.StateMachine('quit')
with wander:
    smach.StateMachine.add('FORWARD', Forward(0.5, 3), transitions={'success': 'BACKWARD'})
    smach.StateMachine.add('LOOK_AROUND', LookAround(0.7), transitions={'object recognized': 'BACKWARD', 'no object recognized': 'FORWARD'})
    smach.StateMachine.add('BACKWARD', Backward(0.5, 3), transitions={'success': 'BACKWARD'})
    # smach.StateMachine.add('STOP', Stop(5), transitions={'success': 'FORWARD' })

if __name__ == '__main__':
    rospy.init_node('wander_state_machine')
    wander.execute()