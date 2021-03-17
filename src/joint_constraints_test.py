#!/usr/bin/env python

from math import pi
import sys

import rospy, actionlib
import moveit_commander

from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, MoveGroupGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

def movegroup_init():
	"""
	Initializes the manipulator and end-effector groups
	@returns Initialized groups
	"""
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('joint_constraints_test', anonymous=True)
	robot = moveit_commander.RobotCommander()

	arm_group = moveit_commander.MoveGroupCommander("manipulator")
	arm_group.allow_replanning(True)
	
	client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	client.wait_for_server()

	# arm_group.set_end_effector_link("wrist_3_link")
	eef_link1 = arm_group.get_end_effector_link()
	print "============ Arm End effector: %s" % eef_link1

	
	arm_group.set_goal_joint_tolerance(0.001)
	arm_group.set_named_target("home")
	plan_arm = arm_group.go()  
	
	joint_pos = arm_group.get_current_joint_values()
	print "joint_goal:", joint_pos

	return arm_group, robot, client
	

def send_joint_trajectory(arm_group, client):
	goal = FollowJointTrajectoryGoal()
	goal.trajectory.joint_names = arm_group.get_active_joints()

	point1 = JointTrajectoryPoint()
	point2 = JointTrajectoryPoint()
	point1.positions = [0.000, -pi/2, pi/2,  0.0, 0.0, 0.0]
	point2.positions = [0.000, -pi/2, pi/2,  pi/2, 0.0, 0.0]

	goal.trajectory.points = [point1, point2]

	goal.trajectory.points[0].time_from_start = rospy.Duration(2.0)
	goal.trajectory.points[1].time_from_start = rospy.Duration(4.0)

	goal.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(1.0)

	client.send_goal(goal)
	print client.wait_for_result()

	
def main():
	try:
		arm_group, robot, client = movegroup_init()
		send_joint_trajectory(arm_group, client)

	except rospy.ROSInterruptException: pass


if __name__ == '__main__': main()
    
