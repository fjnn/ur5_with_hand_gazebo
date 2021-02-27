#! /usr/bin/env python

"""
It looks like I will use move_groups for EE mapping and action server for joint space mapping
TODO: This whole thing can be written in classes. My_Move_Groups(). Don't overcode yet :D
Refer to: http://docs.ros.org/en/indigo/api/pr2_moveit_tutorials/html/planning/scripts/doc/move_group_python_interface_tutorial.html
"""

import sys
import time

import rospy
import moveit_commander
import geometry_msgs.msg
from sensor_msgs.msg import JointState

from Classes.IMU_class_full_arm import IMUsubscriber



#IMU = IMUsubscriber()
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
	rospy.sleep(5)
	arm_group.set_named_target("vertical")
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
    print "============ Arm joint values: ", arm_group_variable_values
    print "click Enter to continue"
    dummy_input = raw_input()
#    arm_group_variable_values[4] = 1.0
#    arm_group.set_joint_value_target(arm_group_variable_values)
    for joint,value in kwargs.items():
        joint_int = joint_names_to_numbers(joint)
        joint_val = value
        arm_group_variable_values[joint_int] = joint_val
        arm_group.set_joint_value_target(arm_group_variable_values)
    plan_arm = arm_group.go() 
    
def rt_joints_mapping(hand_group, arm_group):
    """
    IMU readings will be mapped real time.
    Not move groups but action-server clients will be used
    """
    arm_group.clear_pose_targets()
    hand_group.clear_pose_targets()
    
    arm_group_variable_values = arm_group.get_current_joint_values()
    print "============ Arm joint values: ", arm_group_variable_values
    hand_group_variable_values = hand_group.get_current_joint_values()
    print "============ Hand joint values: ", hand_group_variable_values
    print "click Enter to continue"
    dummy_input = raw_input()
    arm_group_variable_values[4] = 1.0
    arm_group.set_joint_value_target(arm_group_variable_values)
    
    plan = arm_group.plan()
    plan_arm = arm_group.go()  
    
    dummy_input = raw_input()
        

def main():
    try:
        hand_group, arm_group = movegroup_init()
#        rospy.sleep(5)
        joint_space_control(arm_group, wrist_2=1.0, wrist_1=1.0)
#        rt_joints_mapping(hand_group, arm_group)
        sys.exit("done")
#        IMU.init_subscribers_and_publishers()
#        
#        hand_group.set_named_target("handOpen")
#        plan_hand = hand_group.go()  
#        arm_group.set_named_target("home")
#        plan_arm = arm_group.go()  
#        Leap.init_subscribers_and_publishers()
        
#        while not rospy.is_shutdown():
#            now = time.time()
#            prev = 0
#            # print "++++", index
#    #        index += 1  This index is to get 5 sensor readings and compute 1 KF estimation. Check pose_node.py in my_human_pkg for more
#            # print "total: ", (now - start)
#            # print "interval:", (now - prev), "calibration_flag:", IMU.calibration_flag
#            IMU.update()
#            IMU.r.sleep()
#            prev = now
#            Leap.update()
#            Leap.r.sleep()

    except KeyboardInterrupt:
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()




