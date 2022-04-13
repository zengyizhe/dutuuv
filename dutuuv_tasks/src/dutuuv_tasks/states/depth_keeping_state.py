#!/usr/bin/env python3

import rospy
import actionlib
import smach 
from guppy_control.msg import DepthControlAction, DepthControlGoal, DepthControlFeedback

import time

class DepthKeepingState(smach.State):
    def __init__(self, outcomes=['succeeded', 'aborted', 'preempted'], time=300):
        smach.State.__init__(self, outcomes=outcomes)
        self.time = time
        self.action_name = 'depth_control'
        self.action_client = actionlib.SimpleActionClient(self.action_name, DepthControlAction)
        
        self.outcome = None
        # Wait action server for 30 seconds
        if not self.action_client.wait_for_server(timeout=rospy.Duration(300)):
            rospy.logerr('Object Tracking Action Server Timeout')



    def execute(self, userdata):
        
        goal = DepthControlGoal()
        goal.depth_control_action_goal.depth_keeping = True
        self.action_client.send_goal(goal, done_cb=self.done_cb, active_cb=self.active_cb, feedback_cb=self.feedback_cb)
        
        start_time = current_time = time.time()
        while current_time - start_time < self.time:
            current_time = time.time()
            time.sleep(1)
        self.action_client.cancel_goal()
        
        while self.outcome is None:
            time.sleep(1)
        return self.outcome
        
        

    def feedback_cb(self, feedback: DepthControlFeedback):
        pass
        # rospy.loginfo('Current x deviation: {}'.format(feedback.feedback.x_deviation))
        # rospy.loginfo('Current y deviation: {}'.format(feedback.feedback.y_deviation))
        # rospy.loginfo('Current object area ratio: {}'.format(feedback.feedback.current_area_ratio))
        # rospy.loginfo('Goal object area ratio: {}'.format(feedback.feedback.goal_area_ratio))

    def active_cb(self):
        rospy.loginfo('Depth Control Goal active callback')

    def done_cb(self, state: int, result):
        # rospy.loginfo()
        # SUCCEEDED
        rospy.loginfo('state code is {}'.format(state))
        if state == 3:
            self.outcome = 'succeeded'
        # PREEMPTED
        elif state == 2:
            self.outcome ='preempted'
        # ABORTED=4
        elif state == 4:
            self.outcome = 'aborted'
        else: 
            pass



