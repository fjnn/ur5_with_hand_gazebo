#! /usr/bin/env python

'''
This is a mapper class between human and robot wrist motions.
WITHOUT MOVEIT, with action/client-Thats why v2
'''

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

DHmatrices = DHmatrices()

_GYRO_SCALE = 2000 # deg/sec
# # _GYRO_SCALE = 34.9066 # rad/sec

class MapperClass:
	def __init__(self, START_NODE=False, rate=100, mode=0):
		'''
		Initializes a mapper class with 3 modes: JSM, TSM, aTSM
		@params mode=0:JSM, 1:TSM, 2:aTSM   (default = 0)
		'''
		self.mode = mode

		self.human_wrist_joints = Vector3()
		self.hand_pose = Pose()
		self.gyro = Vector3()

		self.speed_gain = Vector3()


		self.Tee_pose = Pose()
		self.Twrist_pose = Pose()  # this looks like wrong calculated
		self.joint_states_openrave = JointState()

		self.jpose = Pose()
		self.tpose = Pose()
		self.apose = Pose()

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
		self.sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, self.sub_hand_pose) # Pub: IMUsubscriber
		self.sub_human_wrist_joints = rospy.Subscriber('/wrist_joints', Vector3, self.sub_wrist_joints)  # Pub: IMUsubscriber
		self.sub_gyro = rospy.Subscriber('/gyro_wrist', Vector3, self.sub_wrist_joints)  # Pub: IMUsubscriber

		self.pub_tee_mapper_goal_pose = rospy.Publisher('/Tee_mapper_goal_pose', Pose, queue_size=1)  # Sub: IKsolver
		self.sub_joints_openrave = rospy.Subscriber('/joint_states_openrave', Vector3, self.sub_joints_openrave)  # Pub: IKsolver
		self.sub_Twrist_pose = rospy.Subscriber('/Twrist_pose', Pose, self.sub_Twrist_pose)  # Pub: IKsolver
		self.sub_Tee_pose = rospy.Subscriber('/Tee_calculated', Pose, self.sub_Tee_pose)  # Pub: IKsolver

		self.pub_jpose = rospy.Publisher('/jpose', Pose, queue_size=1)
		self.pub_tpose = rospy.Publisher('/tpose', Pose, queue_size=1)
		self.pub_apose = rospy.Publisher('/apose', Pose, queue_size=1)

		self.pub_jsm_joints = rospy.Publisher('/jsm_joints', JointState, queue_size=1)
		self.pub_tsm_joints = rospy.Publisher('/tsm_joints', JointState, queue_size=1)
		self.pub_atsm_joints = rospy.Publisher('/atsm_joints', JointState, queue_size=1)
		self.pub_mapper_joints = rospy.Publisher('/mapper_joints', JointState, queue_size=1)

		print "Mapper subs and pubs initialized"

	def sub_Tee_pose(self, msg):
		'''
		Tee_pose is from IKsolver. Not to be used, debug purpose only. /Tee_calculated from OpenRave
		'''
		self.Tee_pose = msg

	def sub_gyro(self, msg):
		'''
		Subscribes the current gyro values and calculate adaptive speed gain
		'''
		c_x = (msg.x/_GYRO_SCALE) * 100
		c_y = (msg.y/_GYRO_SCALE) * 100
		c_z = (msg.z/_GYRO_SCALE) * 100
		self.speed_gain = np.array([c_x, c_y, c_z])


	def sub_Twrist_pose(self, msg):
		'''
		This will go away. Twrist_pose should be always stable. But for now, subscribes IKsolver node
		'''
		self.Twrist_pose = msg


	def sub_hand_pose(self, msg):
		'''
		Subscribes hand_pose {Pose()}, converts it to Tee {Pose()}
		'''
		hand_pose = msg
		param_z = DHmatrices.ee_goal_calculate(hand_pose, self.Twrist_pose)
		self.tpose = Pose(Point(hand_pose.position.x, hand_pose.position.y, param_z), hand_pose.orientation)
		# self.Tee = DHmatrices.pose_to_htm(self.tpose)


	def sub_wrist_joints(self, msg):
		'''
		Subscribes wrist_joints {Vector3()} for JSM
		'''
		self.human_wrist_joints = msg


	def sub_joints_openrave(self):
		'''
		Subscribes the joint states calculated by IKsolver node
		'''
		self.joint_states_openrave = msg


	def jpose_calculate(self, **kwargs):
			"""
			Wrist joints are mapped real time.
			@param **kwargs: [joint_x, value] list
			UR DH: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
			TODO: double quaternions later. HTM now
			NOTE: Not for JSM. JSM sends human wrist joints directly. This is for aTSM calculation
			"""
			for joint,theta in kwargs.items():
				joint_int = MapperClass.joint_names_to_numbers(joint)
				if joint_int == 3:
					rotm3 = DHmatrices.angle_to_rotm(theta, pi/2)
					link_vec3 = DHmatrices.link_calculate(theta, 0.0, 0.13105)
					htm3 = DHmatrices.rotm_to_htm(rotm3, link_vec3)
				elif joint_int == 4:
					rotm4 = DHmatrices.angle_to_rotm(theta, -pi/2)
					link_vec4 = DHmatrices.link_calculate(theta, 0.0, 0.08535)
					htm4 = DHmatrices.rotm_to_htm(rotm4, link_vec4)
				elif joint_int == 5:
					rotm5 = DHmatrices.angle_to_rotm(theta, 0.0)
					link_vec5 = DHmatrices.link_calculate(theta, 0.0, 0.0921)
					htm5 = DHmatrices.rotm_to_htm(rotm5, link_vec5)
				else:
					print "Unknown amount of rotm"
					sys.exit("Returned - plan_joint_space_control")

			htm_final = DHmatrices.matmul(htm3, htm4, htm5)
			# rotm_final = DHmatrices.htm_to_rotm(htm_final)
			quat_final = DHmatrices.htm_to_quat(htm_final)
			self.jpose.position.x = htm_final[0][3]
			self.jpose.position.y = htm_final[1][3]
			self.jpose.position.z = htm_final[2][3]
			self.jpose.orientation.x = quat_final[0]
			self.jpose.orientation.y = quat_final[1]
			self.jpose.orientation.z = quat_final[2]
			self.jpose.orientation.w = quat_final[3]


	def jsm_send_joint_commands(self):
		'''
		No need to Send jpose to IKsolver, publish /mapper_joints for UR_driver
		'''
		self.jsm_joints[4] = self.human_wrist_joints.x
		self.jsm_joints[5] = self.human_wrist_joints.y
		self.jsm_joints[6] = self.human_wrist_joints.z
		self.pub_mapper_joints(self.jsm_joints)
		# self.pub_tee_mapper_goal_pose.publish(self.jsm_pose) --> This is unnecessary. No IK is needed for JSM

	def tsm_send_joint_commands(self):
		'''
		Send tpose to IKsolver, publish /tsm_joints for UR_driver
		'''
		self.tsm_pose = self.tpose
		self.pub_tee_mapper_goal_pose.publish(self.tsm_pose)

	def atsm_send_joint_commands(self):
		'''
		Send tpose to IKsolver, publish /tsm_joints for UR_driver
		'''
		self.tsm_pose = self.apose
		self.pub_tee_mapper_goal_pose.publish(self.atsm_pose)


	def adaptive_tsm_calculate(self, jsm_pose, tsm_pose, speed_gain):
		'''
		Calculate apose based on jpose,tpose and speed_gain
		jpose: Ready, tpose: Ready
		'''
		self.apose.position.x = self.Tee_pose.position.x + self.jpose.position.x + ((self.tpose.position.x - self.jpose.position.x) * self.speed_gain.x)
		self.apose.position.y = self.Tee_pose.position.y + self.jpose.position.y + ((self.tpose.position.y - self.jpose.position.y) * self.speed_gain.y)
		self.apose.position.z = self.Tee_pose.position.z + self.jpose.position.z + ((self.tpose.position.z - self.jpose.position.z) * self.speed_gain.z)
		self.apose.orientation = self.Tee_pose.orientation



	def update(self, elapsed_time):
		'''
		Call this function in every step
		'''
		print "Elapsed time:{}".format(elapsed_time)

		# JSM
		angle_x = float("{:.2f}".format(self.human_wrist_joints.x))
		angle_y = float("{:.2f}".format(self.human_wrist_joints.y))
		angle_z = float("{:.2f}".format(self.human_wrist_joints.z))
		self.jpose_calculate(wrist_1=angle_x, wrist_2=angle_y, wrist_3=angle_z)  #jpose updated
		self.pub_jpose.publish(self.jpose)

		# TSM
		self.pub_tpose.publish(self.tpose)

		# A-TSM
		self.pub_apose.publish(self.apose)

		# Send desired Tee_goal based on selected mode. 
		# IKsolver will publish /joint_states_openrave based on this Tee goal pose
		# Calculated IKsolver joints are published as mapper joints
		if self.mode == 0:
			self.jsm_send_joint_commands()
		elif self.mode == 1:
			self.tsm_send_joint_commands()
		elif self.mode == 2:
			self.atsm_send_joint_commands()
			
		else:
			rospy.signal_shutdown("Wrong selection of human-robot mapper mode")


	@staticmethod
	def joint_names_to_numbers(argument):
		switcher = {
								"shoulder_pan": 0,
								"shoulder_lift": 1,
								"elbow": 2,
								"wrist_1": 3,
								"wrist_2": 4,
								"wrist_3": 5,
								}

		# get() method of dictionary data type returns
		# value of passed argument if it is present
		# in dictionary otherwise second argument will
		# be assigned as default value of passed argument
		return switcher.get(argument, "nothing")
