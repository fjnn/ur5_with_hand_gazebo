#!/usr/bin/env python

from math import pi
import sys

import rospy, actionlib, roslib
import moveit_commander

from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, MoveGroupGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from arm_navigation_msgs.msg import MoveArmResult, MoveArmAction, MoveArmActionGoal, MoveArmGoal, JointConstraint, PositionConstraint, SimplePoseConstraint, OrientationConstraint, Shape


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

	# arm_group.set_end_effector_link("wrist_3_link")
	eef_link1 = arm_group.get_end_effector_link()
	print "============ Arm End effector: %s" % eef_link1

	
	arm_group.set_goal_joint_tolerance(0.001)
	arm_group.set_named_target("home")
	plan_arm = arm_group.go()  
	
	joint_pos = arm_group.get_current_joint_values()
	print "joint_goal:", joint_pos

	return arm_group, robot
	

def send_joint_trajectory(arm_group, client):
	client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	client.wait_for_server()
	
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
	
def move_arm_ompl(arm_group):
	Şimdi rostopic list yaptın, action serverları buldun. MoveGroupActionGoal bulamadım ama
	MoveGroup Action mesajı burada: http://docs.ros.org/en/melodic/api/moveit_msgs/html/action/MoveGroup.html
	Buradan yürürsün artık
	print "Started ompl"
	service = "move_arm"
	client = actionlib.SimpleActionClient(service, MoveArmAction)
	client.wait_for_server()
	print "opml server set"

	goal = MoveArmGoal()
	goal.disable_ik = True
	goal.planner_service_name = "ompl_planning/plan_kinematic_path"
	goal.motion_plan_request.group_name = "arm_group"
	goal.motion_plan_request.num_planning_attempts = 5
	goal.motion_plan_request.allowed_planning_time = rospy.Time.now()+rospy.Duration(5.0)
	goal.motion_plan_request.planner_id = ""

	# addPositionConstraint(goal, x, y, z)
	# addOrientationConstraint(goal)

	constraint = set_joint_constraints(arm_group)
	goal.motion_plan_request.goal_constraints.joint_constraints.append(constraint)
	raw_input("Send goal?")

	# client.send_goal(goal)
	client.send_goal(goal, done_cb=move_arm_completed_cb)
	client.wait_for_result()
	raw_input("Done")
	sys.exit()

def set_joint_constraints(arm_group):
	goal_constraint = Constraints()
	joint_constraint = JointConstraint()
	
	joint_names = arm_group.get_active_joints()
	joint_values = arm_group.get_current_joint_values()
	raw_input("Set joint constraints?")
	for i in range(len(joint_names)):
		joint_constraint.joint_name = joint_names[i]
		joint_constraint.position = joint_values[i]
		if i<3:
			joint_constraint.weight = 10.0 # Closer to zero means less important
		elif i>=3 and i<6:
			joint_constraint.weight = 0.0 # Closer to zero means less important
		else:
			print "Sth is wrong"
		# goal_constraint.joint_constraints.append(joint_constraint)
		print "i:", i
	return joint_constraint
	
def move_arm_completed_cb(status, result):
    if status == GoalStatus.SUCCEEDED:
        print "Succeeded"  
    else:
        print "Failed"
        print result.error_code.val
		



	
def main():
	try:
		arm_group, robot = movegroup_init()
		# send_joint_trajectory(arm_group, client)
		move_arm_ompl(arm_group)

	except rospy.ROSInterruptException: pass


if __name__ == '__main__': main()
    
