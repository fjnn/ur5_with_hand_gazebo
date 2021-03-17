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

from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint


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

	

def main():
	try:
		arm_group, robot = movegroup_init()	
		move_joint(arm_group)

		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			# move_cartesian(arm_group)
			rate.sleep()

	except KeyboardInterrupt:
		moveit_commander.roscpp_shutdown()
		rospy.signal_shutdown("KeyboardInterrupt")
		raise


if __name__ == '__main__': main()




