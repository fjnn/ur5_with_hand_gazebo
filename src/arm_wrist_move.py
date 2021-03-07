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


EE_POSE = Odometry()
WRIST_POSE = Odometry()
GOAL_POSE = Pose()
t = TransformStamped()

IMU = IMUsubscriber()
DHmatrices = DHmatrices()

# _GYRO_SCALE = 2000 # deg/sec
# _GYRO_SCALE = 34.9066 # rad/sec
_GYRO_SCALE = 10.0



def movegroup_init():
	"""
	Initializes the manipulator and end-effector groups
	@returns Initialized groups
	"""
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()

	arm_group = moveit_commander.MoveGroupCommander("arm")
	wrist_group = moveit_commander.MoveGroupCommander("wrist")
	
	# arm_group.set_end_effector_link("wrist_3_link")
	eef_link1 = arm_group.get_end_effector_link()
	print "============ Arm End effector: %s" % eef_link1
	eef_link2 = wrist_group.get_end_effector_link()
	print "============ Wrist End effector: %s" % eef_link2
	
	arm_group.set_named_target("init")
	plan_arm = arm_group.go()  
	wrist_group.set_named_target("init")
	plan_wrist = wrist_group.go()
	wrist_group.get_current_pose().pose
	tool0_pose = wrist_group.get_current_pose().pose
	print "tool0:", tool0_pose  #  rosrun tf tf_echo /world /tool0
	return arm_group, wrist_group, robot
	
def move_arm():
	pass
	
def move_wrist(wrist_group):
	'''
	Constrained TSM test
	tool0: position:
  x: 0.473737345686
  y: -0.836178267497
  z: 1.67958828061
	orientation:
  x: -0.0470110989439
  y: 0.731177822108
  z: -0.315694906137
  w: 0.602914318351

	'''
	goal_c = Constraints()
	goal_c.name = "traj_constraint"
	# Position constraint
	position_c = PositionConstraint()
	# position_c.header = goal_to_append.request.goal_constraints[0].position_constraints[0].header
	# position_c.link_name = goal_to_append.request.goal_constraints[0].position_constraints[0].link_name if link_name == None else link_name
	position_c.link_name = "wrist_3_link"
	position_c.target_point_offset = Vector3(0.0, 0.0, 0.0)
	# position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
	# position_c.constraint_region.primitive_poses.append(goal_pose)
	position_c.weight = 2.0
	goal_c.position_constraints.append(position_c)
	wrist_group.set_path_constraints(goal_c)
	
	# set orientation target
	wrist_ref_frame = wrist_group.get_pose_reference_frame()
	target_pose = PoseStamped()
	target_pose.header.frame_id = wrist_ref_frame
	target_pose.pose.orientation.x = 0.0
	target_pose.pose.orientation.y = 0.0
	target_pose.pose.orientation.z = 0.0
	target_pose.pose.orientation.w = 1.0
	dummy_input = raw_input("Move pose?")
	
	wrist_group.set_pose_target(target_pose)
	wrist_group.go()
	sys.exit("Done")
	
	
	

def movegroup_move_pose(arm_group):
	"""
	Dummy func
	"""
	arm_group.set_named_target("vertical")
	plan_arm = arm_group.go(wait=True)


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
	
	
	arm_group.set_path_constraints(goal_constraint)

	
def plan_task_space_control(arm_group, robot_init, hand_pose):
	waypoints = []
	scale = 1.0
	
	tpose = Pose()
	tpose.position.x = robot_init.position.x + scale * hand_pose.position.y
	tpose.position.y = robot_init.position.y + scale * hand_pose.position.z
	tpose.position.z = robot_init.position.z + scale * hand_pose.position.x
	tpose.orientation = robot_init.orientation
	
	waypoints.append(copy.deepcopy(tpose))
	
	(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

	return tpose

def append_traj_to_move_group_goal(goal_to_append=None, goal_pose=Pose(), link_name=None):
	""" Appends a trajectory_point to the given move_group goal, returns it appended"""
	if goal_to_append == None:
		rospy.logerr("append_trajectory_point_to_move_group_goal needs a goal!")
		return
	#goal_to_append = MoveGroupGoal()
	#rospy.logwarn("goal_to_append is: \n" + str(goal_to_append))
	traj_c = TrajectoryConstraints()
	goal_c = Constraints()
	goal_c.name = "traj_constraint"
	# Position constraint
	position_c = PositionConstraint()
	position_c.header = goal_to_append.request.goal_constraints[0].position_constraints[0].header
	position_c.link_name = goal_to_append.request.goal_constraints[0].position_constraints[0].link_name if link_name == None else link_name
	position_c.constraint_region.primitives.append(SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.01]))
	position_c.constraint_region.primitive_poses.append(goal_pose)
	position_c.weight = 2.0
	goal_c.position_constraints.append(position_c)
	# Orientation constraint
	orientation_c = OrientationConstraint()
	orientation_c.header = goal_to_append.request.goal_constraints[0].position_constraints[0].header
	orientation_c.link_name = goal_to_append.request.goal_constraints[0].position_constraints[0].link_name if link_name == None else link_name
	orientation_c.orientation = goal_pose.orientation
	orientation_c.absolute_x_axis_tolerance = 0.01
	orientation_c.absolute_y_axis_tolerance = 0.01
	orientation_c.absolute_z_axis_tolerance = 0.01
	orientation_c.weight = 1.0
	goal_c.orientation_constraints.append(orientation_c)
	
	traj_c.constraints.append(goal_c)
	goal_to_append.request.trajectory_constraints = traj_c
	
	return goal_to_append   

def main():
    try:
		arm_group, wrist_group, robot = movegroup_init()	
		move_wrist(wrist_group)	
		# set_constraints(arm_group)

		IMU.init_subscribers_and_publishers()
		

		# robot_init = Pose(Point(0.000, 0.000, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0))
		robot_init = arm_group.get_current_pose().pose
		print "robot_init:", robot_init
		inp = raw_input("Click Enter to set new pose?")
		next_pose = Pose(Point(0.42, 0.19, 0.41), Quaternion(0.000, -0.707, -0.707, 0.000))
		arm_group.set_pose_target(next_pose)
		plan = arm_group.go(wait=True)
		print "new pose:", arm_group.get_current_pose().pose
		sys.exit("Done")
		pose_ref_frame = arm_group.get_pose_reference_frame()
		print "============ pose_ref_frame: %s" % pose_ref_frame
		movegroup_move_pose(arm_group)
		robot_move = arm_group.get_current_pose().pose
		print "robot_move:", robot_move
		sys.exit("Done")
		# print "============ Arm current pose: ", robot_init
		# print "click Enter to continue"
		# dummy_input = raw_input()
		# prev = time.time()
		while not rospy.is_shutdown():
			IMU.update()
			if IMU.calibration_flag < 21:
				print "calibration:", IMU.calibration_flag
			else:
			
				# jsm_goal_pose = Pose(Point(0.000, 0.000, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0))
				# tsm_goal_pose = Pose(Point(1.000, 0.000, 0.000), Quaternion(0.000, 0.000, 0.0, 1.0))
				# print "Press Enter for jsm"
				# dummy_input = raw_input()
				angle_x = float("{:.2f}".format(IMU.human_joint_imu.position[0]))
				angle_y = float("{:.2f}".format(IMU.human_joint_imu.position[1]))
				angle_z = float("{:.2f}".format(IMU.human_joint_imu.position[2]))
				jsm_goal_pose = plan_joint_space_control(arm_group, wrist_1=angle_x, wrist_2=angle_y, wrist_3=angle_z)
				print "jsm_goal:", jsm_goal_pose
				
				# print "Press Enter for tsm"
				# dummy_input = raw_input()
				IMU.hand_pos_calculate()
				hand_pose = IMU.tf_wrist
				tsm_goal_pose = plan_task_space_control(arm_group, robot_init, hand_pose)
				print "tsm_goal:", tsm_goal_pose

				# print "Press Enter for adaptive tsm"
				# dummy_input = raw_input()
				gyro = IMU.gyro_wrist
				adaptive_control(arm_group, robot_init, jsm_goal_pose, tsm_goal_pose, gyro)
			IMU.r.sleep()
			
			


    except KeyboardInterrupt:
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()




