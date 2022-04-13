#!/usr/bin/env python3

import rospy
import actionlib
import smach 
from guppy_tasks.states.depth_keeping_state import DepthKeepingState

if __name__ == '__main__':

    rospy.init_node('depth_keeping_state_machine')
    sm = smach.StateMachine(['quit'])
    with sm:
        smach.StateMachine.add('DEPTH_KEEPING', DepthKeepingState(), \
            transitions={'succeeded': 'quit', 'preempted': 'quit', \
                 'aborted': 'quit'})

    sm.execute()
