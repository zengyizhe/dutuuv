#!/usr/bin/env python3
import rospy
import actionlib
import smach 
from dutuuv_tasks.states.fundamental_behaviors import  Down, Up, Stop  
from dutuuv_tasks.states.grasp_state import Grasp

if __name__ == '__main__':

    rospy.init_node('pickup_state_machine')
    sm = smach.StateMachine(['stop_pickup'])
    with sm:

        # wander = smach.StateMachine('stop_wander')
        # with wander:
        #     smach.StateMachine.add('LOOK_AROUND', LookAround(), transitions={'object_detected': 'stop_wander', 'no_object_detected': 'MoveFOward'})
        #     smach.StateMachine.add('MOVE_FORWARD', MoveForward(), transitions={'success': 'TURN_LEFT'})
        #     smach.StateMachine.add('TURN_LEFT', TurnLeft(), transitions={})

        # smach.StateMachine.add('LOOK_FOR_OBJECT', wander(), \
        #     transitions={'detect_object': 'APPROCH_OBJECT'})
        # smach.StateMachine.add('APPROCH_OBJECT', approchObjectState(), transitions={'success', 'PICKUP'})
        smach.StateMachine.add('GRASP', Grasp(), transitions={'success': 'UP'})
        smach.StateMachine.add('UP', Up(0.5, 5), transitions={'success': 'stop_pickup'})

    sm.execute()
