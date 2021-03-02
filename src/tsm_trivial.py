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


EE_POSE = Odometry()
WRIST_POSE = Odometry()
GOAL_POSE = Pose()
t = TransformStamped()

IMU = IMUsubscriber()
#TODO: initiate human model


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
    """
    arm_group.clear_pose_targets()
    
    arm_group_variable_values = arm_group.get_current_joint_values()
    # print "============ Arm joint values: ", arm_group_variable_values
    # print "click Enter to continue"
    # dummy_input = raw_input()
#    arm_group_variable_values[4] = 1.0
    # arm_group.set_joint_value_target(arm_group_variable_values)
    for joint,value in kwargs.items():
        joint_int = joint_names_to_numbers(joint)
        joint_val = value
        arm_group_variable_values[joint_int] = joint_val
        arm_group.set_joint_value_target(arm_group_variable_values)
    plan_arm = arm_group.go(wait=False) 
    
def task_space_control(arm_group, *argv):
	global EE_POSE
	# rosrun tf tf_echo /world /tool0
	pose_goal = Pose()
	pose_goal.orientation.x = 0.707
	pose_goal.orientation.y = -0.000
	pose_goal.orientation.z = -0.000
	pose_goal.orientation.w = 0.707
	pose_goal.position.x = argv[0]
	pose_goal.position.y = argv[1]
	pose_goal.position.z = argv[2]
	arm_group.set_pose_target(pose_goal)

	## Now, we call the planner to compute the plan and execute it.
	plan = arm_group.go(wait=True)
	# Calling `stop()` ensures that there is no residual movement
	arm_group.stop()
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	arm_group.clear_pose_targets()

	## END_SUB_TUTORIAL

	# For testing:
	# Note that since this section of code will not be included in the tutorials
	# we use the class variable rather than the copied state variable
	current_pose = arm_group.get_current_pose().pose
	print "EE_POSE", EE_POSE
	# return all_close(pose_goal, current_pose, 0.01)
	
def cartesian_control(arm_group, *argv):
	waypoints = []
	scale = 1.0
	
	wpose = arm_group.get_current_pose().pose
	wpose.position.z -= scale * 0.1  # First move up (z)
	wpose.position.y += scale * 0.2  # and sideways (y)
	waypoints.append(copy.deepcopy(wpose))
	
	wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
	waypoints.append(copy.deepcopy(wpose))
	
	wpose.position.y -= scale * 0.1  # Third move sideways (y)
	waypoints.append(copy.deepcopy(wpose))
	
	(plan, fraction) = arm_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

	## Now, we call the planner to compute the plan and execute it.
	arm_group.execute(plan, wait=True)
	arm_group.stop()
	# It is always good to clear your targets after planning with poses.
	# Note: there is no equivalent function for clear_joint_value_targets()
	arm_group.clear_pose_targets()

	## END_SUB_TUTORIAL

	# For testing:
	# Note that since this section of code will not be included in the tutorials
	# we use the class variable rather than the copied state variable
	current_pose = arm_group.get_current_pose().pose
	print "EE_POSE", EE_POSE
	# return all_close(pose_goal, current_pose, 0.01)

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
    global t
    
    # create /tf wrist_3_link to /tool0
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "wrist_3_link"
    t.child_frame_id = "tool0"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = -0.082
    t.transform.rotation.x = 0.707
    t.transform.rotation.y = -0.000
    t.transform.rotation.z = -0.000
    t.transform.rotation.w = 0.707
    
    try:
		arm_group = movegroup_init()		
		rospy.Subscriber('/odom_tool0',Odometry,odometryCb_tool0)
		# rospy.sleep(5)
		# joint_space_control(arm_group, wrist_2=0.0, wrist_1=0.0)
		# rt_joints_mapping(hand_group, arm_group)
		# IMU.init_subscribers_and_publishers()
		# rospy.init_node('tsm_trivial',
                    # anonymous=True)
#        
#        hand_group.set_named_target("handOpen")
#        plan_hand = hand_group.go() 
		# print "Going home..."
		# arm_group.set_named_target("home")
		# plan_arm = arm_group.go()  
#        Leap.init_subscribers_and_publishers()

		arm_group_variable_values = arm_group.get_current_joint_values()
		print "============ Arm joint values: ", arm_group_variable_values
		print "click Enter to continue"
		dummy_input = raw_input()
		prev = time.time()
		angle = 0.0
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():
			now = time.time()
			print "Here"
			# print "++++", index
			#        index += 1  This index is to get 5 sensor readings and compute 1 KF estimation. Check pose_node.py in my_human_pkg for more
			# print "total: ", (now - start)
			# print "interval:", (now - prev), "calibration_flag:", IMU.calibration_flag
			# IMU.update()
			# joint_space_control(arm_group, wrist_1=angle_x, wrist_2=angle_y, wrist_3=angle_z)
			wpose = arm_group.get_current_pose().pose
			print "wpose:", wpose
			# print "Enter x_val"
			# x_val = float(raw_input())
			# print "Enter y_val"
			# y_val = float(raw_input())
			# print "Enter z_val"
			# z_val = float(raw_input())
			# task_space_control(arm_group, x_val, y_val, z_val)
			cartesian_control(arm_group)
			
			rospy.sleep(5)


    except KeyboardInterrupt:
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()




