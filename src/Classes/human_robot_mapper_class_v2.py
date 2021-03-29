#! /usr/bin/env python

"""
This is a mapper class between human and robot wrist motions.
WITHOUT MOVEIT
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
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/Classes")
from DH_matrices import DHmatrices
import Kinematics_with_Quaternions as kinematic

## Global variable if Gazebo odom is used
# EE_POSE = Odometry()
# WRIST_POSE = Odometry()
# GOAL_POSE = Pose()
# t = TransformStamped()

_GYRO_SCALE = 2000 # deg/sec
# # _GYRO_SCALE = 34.9066 # rad/sec

class MapperClass:
	def __init__(self, START_NODE=False, rate=100):
		'''
		Initializes a mapper class with 3 modes: JSM, TSM, aTSM
		'''
		self.human_wrist_joints = Vector3()
		self.hand_pose = Pose()
		
		self.Tee_pose = Pose()
		self.joint_states_openrave = JointState()
		
		self.jsm_joints = JointState()
		self.tsm_joints = JointState()
		self.atsm_joints = JointState()		
		
		if START_NODE == True:
			'''
			NOTE: only one node should be called at a time.
			'''
			rospy.init_node("human_robot_mapper_node")
			self.r = rospy.Rate(rate)
			print "Mapper node initialized"
		
		
	def init_subscribers_and_publishers(self):
		self.sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, self.sub_hand_pose)
		self.sub_human_wrist_joints = rospy.Subscriber('/wrist_joints', Vector3, self.sub_wrist_joints)  # Check its publisher
		self.sub_gyro = rospy.Subscriber('/gyro_wrist', Vector3, self.sub_wrist_joints)  # Check its publisher
		
		self.pub = rospy.Publisher('/Tee_goal_pose', Pose, queue_size=1)
		self.sub_joints_openrave = rospy.Subscriber('/joint_states_openrave', Vector3, self.sub_joints_openrave)  # Check its publisher
		
		self.pub_jsm_joints = rospy.Publisher('/jsm_joints', JointState, queue_size=1)
		self.pub_tsm_joints = rospy.Publisher('/tsm_joints', JointState, queue_size=1)
		self.pub_atsm_joints = rospy.Publisher('/atsm_joints', JointState, queue_size=1)
		
		print "Mapper subs and pubs initialized"
		
		
	def sub_hand_pose(self, msg):
		'''
		Subscribes hand_pose {Pose()}, converts it to Tee {Pose()}
		'''
		hand_pose = msg
		param_z = DHmatrices.ee_goal_calculate(hand_pose, self.ur5_wrist_pose)
		self.Tee_pose = Pose(Point(hand_pose.position.x, hand_pose.position.y, param_z), hand_pose.orientation)
		# self.Tee = DHmatrices.pose_to_htm(Tee_pose)
			
	def sub_wrist_joints(self, msg):
		'''
		Subscribes wrist_joints {Vector3()} for JSM
		'''
		self.human_wrist_joints = msg
	
	# def jsm_calculate(self, wrist_joints):
		# pass
		
	# def tsm_calculate(self, hand_pose):
		# pass
		
	# def adaptive_tsm_calculate(self, jsm_pose, tsm_pose, speed_gain):
		# pass
		
	def update(self, elapsed_time):
		'''
		Call this function in every step
		'''
		print "Elapsed time:{}".format(elapsed_time)
		
		# jsm_pose = self.jsm_calculate(self.human_wrist_joints)
		# tsm_pose = self.tsm_calculate(self.Tee)
		# a_tsm_pose = self.adaptive_tsm_calculate(self, jsm_pose, tsm_pose, speed_gain)
	
		# self.pub.publish(self.a_tsm_pose)
		# self.pub.publish(self.tsm_pose)
		# self.pub.publish(self.jsm_pose)








