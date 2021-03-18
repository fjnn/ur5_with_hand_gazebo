#! /usr/bin/env python

"""
It looks like I will use move_groups for EE mapping and action server for joint space mapping
TODO: This whole thing can be written in classes. My_Move_Groups(). Don't overcode yet :D
Refer to: http://docs.ros.org/en/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
"""

import sys
import time
import copy
from math import pi
import numpy as np


import rospy
import moveit_commander
from geometry_msgs.msg import Pose, PoseStamped
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, MoveGroupGoal


from Classes.IMU_class_elbow_angle import IMUsubscriber
import Classes.Kinematics_with_Quaternions as kinematic
from Classes.DH_matrices import DHmatrices



def movegroup_init():
	"""
	Initializes the manipulator and end-effector groups
	@returns Initialized groups
	"""
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
	robot = moveit_commander.RobotCommander()

	arm_group = moveit_commander.MoveGroupCommander("manipulator")
	arm_group.allow_replanning(True)

	# arm_group.set_end_effector_link("wrist_3_link")
	eef_link1 = arm_group.get_end_effector_link()
	print "============ Arm End effector: %s" % eef_link1

	
	arm_group.set_goal_joint_tolerance(0.001)
	arm_group.set_named_target("home")
	plan_arm = arm_group.go()  

	return arm_group, robot
	
def move_cartesian(arm_group):
	waypoints = []
	scale = 1.0

	wpose = arm_group.get_current_pose().pose
	wpose.position.z -= scale * 0.1  # First move up (z)
	wpose.position.y += scale * 0.2  # and sideways (y)
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
	waypoints.append(copy.deepcopy(wpose))

	wpose.position.y -= scale * 0.1  # Third move sideways (y)
	waypoints.append(copy.deepcopy(wpose))

	# We want the Cartesian path to be interpolated at a resolution of 1 cm
	# which is why we will specify 0.01 as the eef_step in Cartesian
	# translation.  We will disable the jump threshold by setting it to 0.0 disabling:
	(plan, fraction) = arm_group.compute_cartesian_path(
																		 waypoints,   # waypoints to follow
																		 0.01,        # eef_step
																		 0.0)         # jump_threshold
	 
	arm_group.execute(plan, wait=True)
	
def move_joint(arm_group):
	"""
	Move wrist joints and get ee_pose.
	Later I will use this info only to move wrist
	"""
	joint_goal = arm_group.get_current_joint_values()
	print "joint_goal:", joint_goal
	# ============ Arm End effector: tool0
	# joint_goal: [0.007552534881796191, -1.4923353069701557, 1.47499200036974, -0.07242291329326278, -0.09289451682609595, 0.010221995571863651]

	joint_goal[4] = pi/2
	arm_group.go(joint_goal, wait=True)
	print "Final joint values:", arm_group.get_current_joint_values()
	

pose_goal = Pose()
joint_constraint = JointConstraint()
moveit_goal = MoveGroupGoal()
goal_constraint = Constraints()
def move_ee_pose(arm_group):
	"""
	Move end-effector to specified pose
	"""
	current_pose = arm_group.get_current_pose(end_effector_link="tool0")
	print "current_pose:", current_pose
	# pose home
  # position:
    # x: 0.422562465178
    # y: 0.191221247515
    # z: 0.418782506626
  # orientation:
    # x: -0.000369305197861
    # y: 0.707106486605
    # z: 0.707106290394
    # w: 0.000987066022963
    
  # pose home + joint_goal[4] = pi/2
  # position:
    # x: 0.504844264658
    # y: 0.108320076232
    # z: 0.418621428128
  # orientation:
    # x: 0.500459099487
    # y: 0.498982558429
    # z: 0.49980940138
    # w: 0.500747100257
	joint_values = arm_group.get_current_joint_values()
	pose_goal.position.x = 0.504
	pose_goal.position.y = 0.108
	pose_goal.position.z = 0.418
	arm_group.set_position_target([pose_goal.position.x, pose_goal.position.y, pose_goal.position.z])
	# arm_group.set_pose_target(pose_goal)
	raw_input("Set constraints?")
	print "Const-0:", arm_group.get_known_constraints()
	joint_names = arm_group.get_active_joints()
	print "joint_names:", joint_names
	for i in range(len(joint_names)):
		if i<3:
			joint_constraint.joint_name = joint_names[i]
			joint_constraint.position = joint_values[i]
			joint_constraint.weight = 10.0 # Closer to zero means less important
			goal_constraint.joint_constraints.append(joint_constraint)
		elif i>=3:
			joint_constraint.weight = 0.0 # Closer to zero means less important
		else:
			print "Sth is wrong"
		print "i:", i
	
	print "Const-1:", arm_group.get_known_constraints()
	moveit_goal.request.goal_constraints.append(goal_constraint)
	print "Const-2:", arm_group.get_known_constraints()
	moveit_goal.request.goal_constraints.append(goal_constraint)
	moveit_goal.request.num_planning_attempts = 1
	moveit_goal.request.allowed_planning_time = 5.0
	moveit_goal.planning_options.plan_only = False
	moveit_goal.planning_options.planning_scene_diff.is_diff = True
	moveit_goal.request.group_name = arm_group
	
	# arm_group.motion_plan_request.goal_constraints.joint_constraints.append(goal_constraint)
	arm_group.set_path_constraints(goal_constraint)
	print "Const-3:", arm_group.get_known_constraints()
	arm_group.go(pose_goal, wait=True)
	print "Final joint values:", arm_group.get_current_joint_values()
	raw_input("Done")
	sys.exit()
	
def set_constraints(arm_group):
	goal_constraint = Constraints()
	joint_values = [0.0, -1.5, 1.5]
	joint_names = arm_group.get_active_joints()
	print "joint_names:", joint_names
	for i in range(3):
		joint_constraint = JointConstraint()
		joint_constraint.joint_name = joint_names[i]
		joint_constraint.position = joint_values[i]
		joint_constraint.weight = 1.0
		goal_constraint.joint_constraints.append(joint_constraint)
		
	# set_workspace(self, ws): """ Set the workspace for the robot as either [], [minX, minY, maxX, maxY] or [minX, minY, minZ, maxX, maxY, maxZ] """
	
	
	arm_group.set_trajectory_constraints(goal_constraint)
	# arm_group._g.request.goal_constraints.append(goal_constraint)
	# arm_group._goal.planning_options.planning_scene_diff.robot_state.is_diff = True
		
	# print "path const:", arm_group.get_path_constraints()
	print "known const:", arm_group.get_known_constraints()
	sys.exit("Done")

def main():
	try:
		arm_group, robot = movegroup_init()	
		set_constraints(arm_group)
		# move_joint(arm_group)
		move_ee_pose(arm_group)

		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			# move_cartesian(arm_group)
			rate.sleep()

	except KeyboardInterrupt:
		moveit_commander.roscpp_shutdown()
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()



