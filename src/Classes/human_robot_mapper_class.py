#! /usr/bin/env python

"""
This is a mapper class between human and robot wrist motions.
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

# sys.path.append("/home/gizem/catkin_ws/src/ur5_with_hand_gazebo/src/")
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
		print "here"
		moveit_commander.roscpp_initialize(sys.argv)
		if START_NODE==True:
			self.start_node(rate)
		self.robot = moveit_commander.RobotCommander()

		self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
		self.arm_group.set_named_target("home")
		plan_arm = self.arm_group.go()  
		
		self.ur5_wrist_pose = Pose(Point(0.422, 0.016, 0.513), Quaternion(-0.000, 1.000, -0.000, -0.000))  ## This shouldn't change actually. You can make it constant. # rosrun tf tf_echo /world /tool0
			# - Translation: [0.422, 0.016, 0.513]
			# - Rotation: in Quaternion [-0.000, 1.000, -0.000, -0.000]
							# in RPY (radian) [3.142, -0.000, -3.141]
							# in RPY (degree) [180.000, -0.004, -179.987]
		self.ur5_tool_pose = Pose(Point(0.422, 0.192, 0.418), Quaternion(-0.000, 0.707, 0.707, -0.000))  ## rosrun tf tf_echo /world /tool0
			# - Translation: [0.422, 0.192, 0.418]
			# - Rotation: in Quaternion [-0.000, 0.707, 0.707, -0.000]
            # in RPY (radian) [1.571, -0.000, -3.141]
            # in RPY (degree) [90.000, -0.001, -179.989]
    
    self.human_wrist_joints = Vector3()
		self.Tee_pose = Pose() ## To be sent IKsolver node
		
		self.start_time = rospy.Time.now()
		print "Mapper initialized"
				
		# return arm_group, robot
		
	def start_node(self, rate):
		'''
		NOTE: only one node should be calling this function.
		'''
		rospy.init_node('human_robot_mapper_node', anonymous=False)
		self.r = rospy.Rate(rate)
		print "Mapper node initialized"
		
		
	def init_subscribers_and_publishers(self):
		self.pub = rospy.Publisher('/Tee_goal_pose', Pose, queue_size=1)
		# self.pub = rospy.Publisher('/tsm_pose', Pose, queue_size=1)
		# self.pub = rospy.Publisher('/jsm_pose', Pose, queue_size=1)
		self.sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, self.callback_hand_pose)
		self.sub_human_wrist_joints = rospy.Subscriber('/wrist_joints', Vector3, self.callback_wrist_joints)  # Check its publisher
		print "Mapper subs and pubs initialized"
		
	def callback_hand_pose(self, msg):
		'''
		Subscribes hand_pose {Pose()}, converts it to Tee {Pose()}
		'''
		hand_pose = msg
		param_z = DHmatrices.ee_goal_calculate(hand_pose, self.ur5_wrist_pose)
		self.Tee_pose = Pose(Point(hand_pose.position.x, hand_pose.position.y, param_z), hand_pose.orientation)
		# self.Tee = DHmatrices.pose_to_htm(Tee_pose)
			
	def callback_wrist_joints(self, msg):
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
		
	def update(self):
		'''
		Call this function in every step
		'''
		self.curr_time = rospy.Time.now()
		elapsed_time = (self.curr_time - self.start_time)/1000000.0
		print "Elapsed time:{}".format(elapsed_time)
		
		# jsm_pose = self.jsm_calculate(self.human_wrist_joints)
		# tsm_pose = self.tsm_calculate(self.Tee)
		# a_tsm_pose = self.adaptive_tsm_calculate(self, jsm_pose, tsm_pose, speed_gain)
	
		# self.pub.publish(self.a_tsm_pose)
		# self.pub.publish(self.tsm_pose)
		# self.pub.publish(self.jsm_pose)








