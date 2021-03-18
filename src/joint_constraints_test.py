#!/usr/bin/env python

from math import pi
import sys

import rospy, actionlib, roslib
import moveit_commander

from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, MoveGroupAction, MoveGroupActionGoal, MoveGroupGoal
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
	

def send_joint_trajectory(arm_group):
	'''
	Create a FollowJointTrajectoryAction client and sen joint trajectory.
	Note: works fine
	@params arm_group:  moveit_commander.MoveGroupCommander("manipulator_name")
	'''
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
	
def send_point_target(arm_group, client):
	'''
	Try to make a control_msgs/PointHeadActionGoal action from moveit or something. Try to send here
	'''
	# client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
	# client.wait_for_server()
	
	# goal = FollowJointTrajectoryGoal()
	# goal.trajectory.joint_names = arm_group.get_active_joints()

	# point1 = JointTrajectoryPoint()
	# point2 = JointTrajectoryPoint()
	# point1.positions = [0.000, -pi/2, pi/2,  0.0, 0.0, 0.0]
	# point2.positions = [0.000, -pi/2, pi/2,  pi/2, 0.0, 0.0]

	# goal.trajectory.points = [point1, point2]

	# goal.trajectory.points[0].time_from_start = rospy.Duration(2.0)
	# goal.trajectory.points[1].time_from_start = rospy.Duration(4.0)

	# goal.trajectory.header.stamp = rospy.Time.now()+rospy.Duration(1.0)

	# client.send_goal(goal)
	# print client.wait_for_result()
	pass
	
def plan_arm_ompl(arm_group):
	print "Started ompl"
	client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
	client.wait_for_server()
	print "opml server set"
	
	move_group_action_goal = MoveGroupActionGoal()
	goal = MoveGroupGoal()
	
	goal.request.goal_constraints = set_joint_constraints(arm_group)
	
	# goal_constraint = set_joint_constraints(arm_group)
	raw_input("Send goal?")


	move_group_action_goal.goal = goal
	client.send_goal(goal) # field goal.request.goal_constraints must be a list or tuple type
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
		if i<3:
			joint_constraint.joint_name = joint_names[i]
			joint_constraint.position = joint_values[i]
			joint_constraint.weight = 10.0 # Closer to zero means less important
		elif i>=3 and i<6:
			joint_constraint.weight = 0.0 # Closer to zero means less important
		else:
			print "Sth is wrong"
		goal_constraint.joint_constraints.append(joint_constraint)
		print "i:", i
	return goal_constraint
	
def move_arm_completed_cb(status, result):
    if status == GoalStatus.SUCCEEDED:
        print "Succeeded"  
    else:
        print "Failed"
        print result.error_code.val

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
	print "Initial joint values:", arm_group.get_current_joint_values()
	pose_goal.position.x = 0.504
	pose_goal.position.y = 0.108
	pose_goal.position.z = 0.418
	arm_group.set_position_target([pose_goal.position.x, pose_goal.position.y, pose_goal.position.z])
	

	# arm_group.set_path_constraints(goal_constraint)
	print "Const-3:", arm_group.get_known_constraints()
	arm_group.go(pose_goal, wait=True)
	print "Final joint values:", arm_group.get_current_joint_values()


	
def main():
	try:
		arm_group, robot = movegroup_init()
		plan_arm_ompl(arm_group)
		move_ee_pose(arm_group)
		
		raw_input("Done")
		sys.exit()
		

	except rospy.ROSInterruptException: pass


if __name__ == '__main__': main()
    
