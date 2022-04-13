#!/usr/bin/env python3
import rospy
import actionlib
import smach 
from dutuuv_tasks.states.fundamental_behaviors import Forward, Backward, Down, Up, Stop  

wander = smach.StateMachine('quit')
with wander:
    smach.StateMachine.add('FORWARD', Forward(0.5, 3), transitions={'success': 'BACKWARD'})
    smach.StateMachine.add('BACKWARD', Backward(0.5, 3), transitions={'success': 'UP'})
    smach.StateMachine.add('UP', Up(0.5, 2), transitions={'success': 'DOWN'})
    smach.StateMachine.add('DOWN', Down(0.5, 2), transitions={'success': 'STOP'})
    smach.StateMachine.add('STOP', Stop(5), transitions={'success': 'FORWARD' })

if __name__ == '__main__':
    rospy.init_node('wander_state_machine')
    wander.execute()