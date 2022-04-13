
#!/usr/bin/env python3

import rospy
import actionlib
import smach 
from guppy_control.msg import ObjectTrackingAction, ObjectTrackingGoal, ObjectTrackingFeedback


class ObjectTrackingState(smach.State):
    def __init__(self, goal: ObjectTrackingGoal, outcomes=['succeeded', 'aborted', 'preempted']):
        smach.State.__init__(self, outcomes=outcomes)

        self.action_client = actionlib.SimpleActionClient('object_tracking', ObjectTrackingAction)
        
        # Wait action server for 30 seconds
        if not self.action_client.wait_for_server(timeout=rospy.Duration(60)):
            rospy.logerr('Object tracking action server is not working')

        self.goal = goal

    def execute(self, userdata):
        self.action_client.send_goal(self.goal, feedback_cb=self.feedback_cb)
        self.action_client.wait_for_result()
        
        result_state = self.action_client.get_state()
        
        # actionlib PREEMPTED=2
        if result_state == 2: 
            return 'preempted'
        
        # actionlib SUCCEEDED=3
        elif result_state == 3:
            return 'succeeded'
        
        # actionlib ABORTED=4
        elif result_state == 4:
            return 'aborted'


    def feedback_cb(self, feedback: ObjectTrackingFeedback):
        print('Current x deviation: {}'.format(feedback.feedback.x_deviation))
        print('Current y deviation: {}'.format(feedback.feedback.y_deviation))
        print('Current object area ratio: {}'.format(feedback.feedback.current_area_ratio))
        print('Goal object area ratio: {}'.format(feedback.feedback.goal_area_ratio))
