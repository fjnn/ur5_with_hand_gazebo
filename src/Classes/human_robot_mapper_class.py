#! /usr/bin/env python

"""
This is a mapper class between human and robot wrist motions.
TODO: Seperate MoveIT robot class.
"""

import sys
import time
import copy
from math import pi
import numpy as np

import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

import Kinematics_with_Quaternions as kinematic


EE_POSE = Odometry()
WRIST_POSE = Odometry()
GOAL_POSE = Pose()
t = TransformStamped()

_GYRO_SCALE = 2000 # deg/sec
# # _GYRO_SCALE = 34.9066 # rad/sec

class MapperClass:
	def __init__(self, START_NODE=False, rate=100):
		'''
		Initializes a mapper class with 3 modes: JSM, TSM, aTSM
		'''
		print "here"
		moveit_commander.roscpp_initialize(sys.argv)
		if START_NODE==True:
			self.start_node(rate)
		self.robot = moveit_commander.RobotCommander()

		self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
		self.arm_group.set_named_target("vertical")
		plan_arm = self.arm_group.go()  
		
		self.start_time = rospy.Time.now()
		print "Mapper initialized"
				
		# return arm_group, robot
		
	def start_node(self, rate):
		'''
		NOTE: only one node should be calling this function.
		'''
		rospy.init_node('human_robot_mapper_node', anonymous=False)
		self.r = rospy.Rate(rate)
		
		
	def init_subscribers_and_publishers(self):
		self.pub = rospy.Publisher('/ur5_joint_position_cmd', JointState, queue_size=1)
		self.sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, self.callback_hand_pose)
		self.sub_human_wrist_joints = rospy.Subscriber('/wrist_joints', Vector3, self.callback_wrist_joints)  # Check its publisher
		print "Mapper subs and pubs initialized"
		
	def callback_hand_pose(self, msg):
		pass
		
	# def callback_wrist_joints(self, msg):
		# pass
	
	# def jsm_calculate(self, wrist_joints):
		# pass
		
	# def tsm_calculate(self, hand_pose):
		# pass
		
	# def adaptive_tsm_calculate(self, jsm_pose, tsm_pose, speed_gain):
		# pass
		
	def update(self):
		'''
		Call this function in every step
		'''
		self.curr_time = rospy.Time.now()
		elapsed_time = (self.curr_time - self.start_time)/1000000.0
		print "Elapsed time:{}".format(elapsed_time)








