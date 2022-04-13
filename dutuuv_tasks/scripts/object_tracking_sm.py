#!/usr/bin/env python3

from guppy_tasks.states.object_tracking_state import ObjectTrackingState

import rospy
import actionlib
import smach 
from guppy_control.msg import ObjectTrackingAction, ObjectTrackingGoal, ObjectTrackingFeedback

goal = ObjectTrackingGoal()
goal.goal.object_name = 'yellow_ball'
goal.goal.object_area_ratio = 0.02  


rospy.init_node('object_tracking_sm')
sm = smach.StateMachine(['aborted'])
with sm:
    smach.StateMachine.add('TRACK_OBJECT', ObjectTrackingState(goal), \
        transitions={'succeeded': 'aborted', 'preempted': 'aborted', 'aborted': 'aborted'})


sm.execute()
