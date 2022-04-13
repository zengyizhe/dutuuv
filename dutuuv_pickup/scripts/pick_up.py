#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose

joint_state_topic = ['joint_states:=/dlut_auv/joint_states']

moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('pick_up')
moveit_commander.roscpp_initialize(sys.argv)

group_name = 'my_manipulator_arm'
move_group = moveit_commander.MoveGroupCommander(group_name)
robot = moveit_commander.RobotCommander()

_ = move_group.get_pose_reference_frame()
print(_)
_ = move_group.get_planning_frame()
print(_)
# move_group.set_pose_reference_frame('base_link')
# _ = move_group.get_pose_reference_frame()
# print(_)
# move_group.set_planning_frame('shoulder_link')
# _ = move_group.get_planning_frame()
# print(_)
# We can get the name of the reference frame for this robot:
# planning_frame = move_group.get_planning_frame()
# print("============ Planning frame: %s" % planning_frame)

# # move_group.set_pose_reference_frame("base_link")
# planning_frame = move_group.get_planning_frame()
# print("============ Planning frame: %s" % planning_frame)

# # We can also print the name of the end-effector link for this group:
# eef_link = move_group.get_end_effector_link()
# print("============ End effector link: %s" % eef_link)

# # We can get a list of all the groups in the robot:
# group_names = robot.get_group_names()
# print("============ Available Planning Groups:", robot.get_group_names())

# # Sometimes for debugging it is useful to print the entire state of the
# # robot:
# print("============ Printing robot state")
# print(robot.get_current_state())
# print("")
#  0, 0.8509035, 0, 0.525322 ]

# current_pose = move_group.get_current_pose().pose
# print(move_group.get_current_pose().pose)

# move_group.set_goal_orientation_tolerance(0.5)
# move_group.set_goal_position_tolerance(0.03)



pose_goal = Pose()

pose_goal.position.x = 0.5 


pose_goal.position.y = -0.2 


pose_goal.position.z = -0.23 

pose_goal.orientation.y = 0.8509035

pose_goal.orientation.w= 0.525322

move_group.set_pose_target(pose_goal)
move_group.go(wait=True)



move_group.set_named_target('home')
move_group.go()

