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
from tf.transformations import euler_from_matrix as m2e


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



def movegroup_init():
	"""
	Initializes the manipulator and end-effector groups
	@returns Initialized groups
	"""
	moveit_commander.roscpp_initialize(sys.argv)
	robot = moveit_commander.RobotCommander()

	arm_group = moveit_commander.MoveGroupCommander("manipulator")
	arm_group.set_named_target("home")
	plan_arm = arm_group.go()  
	return arm_group

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


def joint_space_control(arm_group, **kwargs):
    """
    IMU readings will be mapped real time.
    @param arm_group: manipulator group name
    @param **kwargs: [joint_x, value] list
    UR DH: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    TODO: double quaternions later. HTM now
    """
    global DHmatrices
    arm_group.clear_pose_targets()
    
    arm_group_variable_values = arm_group.get_current_joint_values()
    # print "============ Arm joint values: ", arm_group_variable_values
    # print "click Enter to continue"
    # dummy_input = raw_input()
#    arm_group_variable_values[4] = 1.0
    # arm_group.set_joint_value_target(arm_group_variable_values)
    gpose = Pose()
    
    for joint,theta in kwargs.items():
        joint_int = joint_names_to_numbers(joint)
        if joint_int == 3:
					rotm3 = DHmatrices.angle_to_rotm(theta, pi/2):
					link_vec3 = DHmatrices.link_calculate(theta, 0.0, 0.13105)
					htm3 = DHmatrices.rotm_to_htm(rotm3, link_vec3)
				elif joint_int == 4:
					rotm4 = DHmatrices.angle_to_rotm(theta, -pi/2):
					link_vec4 = DHmatrices.link_calculate(theta, 0.0, 0.08535)
					htm4 = DHmatrices.rotm_to_htm(rotm4, link_vec4)
				elif joint_int == 5:
					rotm5 = DHmatrices.angle_to_rotm(theta, 0.0)
					link_vec5 = DHmatrices.link_calculate(theta, 0.0, 0.0921)
					htm5 = DHmatrices.rotm_to_htm(rotm5, link_vec5)
				else:
					print "Unknown amount of rotm"
    
    htm_final = DHmatrices(rotm3, rotm4, rotm5)
		rotm_final = DHmatrices(htm_final)
    gpose.position.x = htm_final[0][3]
    gpose.position.y = htm_final[1][3]
    gpose.position.z = htm_final[2][3]
    gpose.orientation = # I need quaternion here.
    return gpose
	

def cartesian_control_with_IMU(arm_group, robot_init, hand_pose, *argv):
	waypoints = []
	scale = 1.0
	
	wpose = Pose()
	wpose.position.x = robot_init.position.x + scale * hand_pose.position.y
	wpose.position.y = robot_init.position.y + scale * hand_pose.position.z
	wpose.position.z = robot_init.position.z + scale * hand_pose.position.x
	wpose.orientation = robot_init.orientation
	# wpose.orientation = kinematic.q_multiply(robot_init.orientation, hand_pose.orientation)
	
	waypoints.append(copy.deepcopy(wpose))
	
	(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

	return wpose
	# arm_group.execute(plan, wait=True)
	# arm_group.stop()
	# arm_group.clear_pose_targets()
	
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




