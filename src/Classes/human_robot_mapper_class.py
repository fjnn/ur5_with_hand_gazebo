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


from Classes.IMU_class_elbow_angle import IMUsubscriber
import Classes.Kinematics_with_Quaternions as kinematic
from Classes.DH_matrices import DHmatrices


EE_POSE = Odometry()
WRIST_POSE = Odometry()
GOAL_POSE = Pose()
t = TransformStamped()

IMU = IMUsubscriber()
DHmatrices = DHmatrices()

_GYRO_SCALE = 2000 # deg/sec
# _GYRO_SCALE = 34.9066 # rad/sec

class MapperClass:
	def __init__(self, START_NODE=False, rate=100):
		'''
		Initializes a mapper class with 3 modes: JSM, TSM, aTSM
		'''
		moveit_commander.roscpp_initialize(sys.argv)
		if START_NODE == True:
			self.start_node()
		self.robot = moveit_commander.RobotCommander()
		
		self.arm_group = moveit_commander.MoveGroupCommander("manipulator")
		self.arm_group.allow_replanning(True)

		# arm_group.set_end_effector_link("wrist_3_link")
		self.eef_link1 = arm_group.get_end_effector_link()
		print "============ Arm End effector: %s" % self.eef_link1

		
		self.arm_group.set_goal_joint_tolerance(0.001)
		self.arm_group.set_named_target("home")
		plan_arm = self.arm_group.go()  
		
		self.joint_pos = self.arm_group.get_current_joint_values()
		print "joint_goal:", self.joint_pos
		
		self.prev_time = rospy.Time.now()
		self.curr_time = rospy.Time.now()
		
		# return arm_group, robot
		
	def start_node(self):
		'''
		NOTE: only one node should be calling this function.
		'''
		rospy.init_node('human_robot_mapper_node', anonymous=False)
		
		
	def init_subscribers_and_publishers(self):
		pass
		
	def callback_hand_pose(self, msg):
		pass
	
	def jsm_calculate(self, wrist_joints):
		pass
		
	def tsm_calculate(self, hand_pose):
		pass
		
	def adaptive_tsm_calculate(self, jsm_pose, tsm_pose, speed_gain):
		pass
		
	def update(self):
		'''
		Call this function in every step
		'''
		self.curr_time = rospy.Time.now()
		print "Elapsed time:", self.curr_time - self.prev_time



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


def plan_joint_space_control(arm_group, **kwargs):
	"""
	IMU readings will be mapped real time.
	@param arm_group: manipulator group name
	@param **kwargs: [joint_x, value] list
	UR DH: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
	TODO: double quaternions later. HTM now
	"""
	global DHmatrices
	jpose = Pose()
	
	arm_group.clear_pose_targets()
	
	arm_group_variable_values = arm_group.get_current_joint_values()
	
	for joint,theta in kwargs.items():
		
			print "----------------"
			joint_int = joint_names_to_numbers(joint)
			print "joint:", joint_int
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
				print "rotm5", rotm5
				link_vec5 = DHmatrices.link_calculate(theta, 0.0, 0.0921)
				print "link_vec5", link_vec5
				htm5 = DHmatrices.rotm_to_htm(rotm5, link_vec5)
			else:
				print "Unknown amount of rotm"
	
	htm_final = DHmatrices.matmul(rotm3, rotm4, rotm5)
	# rotm_final = DHmatrices.htm_to_rotm(htm_final)
	quat_final = DHmatrices.htm_to_quat(htm_final)
	jpose.position.x = htm_final[0][3]
	jpose.position.y = htm_final[1][3]
	jpose.position.z = htm_final[2][3]
	jpose.orientation.x = quat_final[0]
	jpose.orientation.y = quat_final[1]
	jpose.orientation.z = quat_final[2]
	jpose.orientation.w = quat_final[3]
	return jpose
	

def plan_task_space_control(arm_group, robot_init, hand_pose, *argv):
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

	
def adaptive_control(robot_init, jsm_goal_pose, tsm_goal_pose, gyro):
	global _GYRO_SCALE
	waypoints = []
	# Read IMU.gyro
	print "IMU gyro", gyro
	# D gyro min-max range
	# Calculate adaptive gain
	c_x = (gyro.x/_GYRO_SCALE) * 100
	c_y = (gyro.y/_GYRO_SCALE) * 100
	c_z = (gyro.z/_GYRO_SCALE) * 100
	
	apose = Pose()
	apose.position.x = robot_init.position.x + jsm_goal_pose.position.x + ((tsm_goal_pose.position.x -jsm_goal_pose.position.x) * c_x)
	apose.position.y = robot_init.position.y + jsm_goal_pose.position.y + ((tsm_goal_pose.position.y -jsm_goal_pose.position.y) * c_y)
	apose.position.z = robot_init.position.z + jsm_goal_pose.position.z + ((tsm_goal_pose.position.z -jsm_goal_pose.position.z) * c_z)
	apose.orientation = robot_init.orientation
	# wpose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose.orientation)
	
	waypoints.append(copy.deepcopy(wpose))
	
	(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

	# arm_group.execute(plan, wait=True)
	# arm_group.stop()
	# arm_group.clear_pose_targets()
	


def odometryCb_tool0(msg):
	global EE_POSE, t
	'''
	msg: world to wrist_3_link
	EE_POSE should be calculated here according to: 
	
	rosrun tf tf_echo /tool0 /wrist_3_link
	At time 0.000
	- Translation: [0.000, -0.000, -0.082]
	- Rotation: in Quaternion [0.707, -0.000, -0.000, 0.707]
							in RPY (radian) [1.571, -0.000, -0.000]
							in RPY (degree) [90.000, -0.000, -0.000]
	'''

	EE_POSE = msg
	# print msg.pose.pose
	    

def main():
    
    try:
		arm_group = movegroup_init()		
		# rospy.Subscriber('/odom_tool0',Odometry,odometryCb_tool0)
		# rospy.sleep(5)

		IMU.init_subscribers_and_publishers()

		robot_init = arm_group.get_current_pose().pose
		print "============ Arm current pose: ", robot_init
		print "click Enter to continue"
		dummy_input = raw_input()
		prev = time.time()
		while not rospy.is_shutdown():
			if IMU.calibration_flag < 21:
				print "calibration:", IMU.calibration_flag
			else:
				# robot_init = Pose(Point(-0.175, 0.000, -0.095), Quaternion(0.000, 0.000, -0.707, 0.707))				print "robot_init:", robot_init
				IMU.hand_pos_calculate()
				GOAL_POSE = IMU.tf_wrist
				print "GOAL_POSE", GOAL_POSE
				adaptive_control(robot_init, jsm_goal_pose, tsm_goal_pose, gyro)
				# wpose = arm_group.get_current_pose().pose
				# print "wpose:", wpose
				# print "Enter x_val"
				# x_val = float(raw_input())
				# print "Enter y_val"
				# y_val = float(raw_input())
				# print "Enter z_val"
				# z_val = float(raw_input())
				# cartesian_control_with_IMU(arm_group, robot_init, GOAL_POSE, x_val, y_val, z_val)
				# task_space_control(arm_group, x_val, y_val, z_val)
			IMU.update()
			IMU.r.sleep()
			
			


    except KeyboardInterrupt:
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()




