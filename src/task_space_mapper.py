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
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

from Classes.IMU_class_elbow_angle import IMUsubscriber

HAND_POSE = Odometry()
WRIST_POSE = Odometry()
GOAL_POSE = Pose()

IMU = IMUsubscriber()
#Leap = LeapSubscriber()
#TODO: initiate human model


def movegroup_init():
    """
    Initializes the manipulator and end-effector groups
    @returns Initialized groups
    """
    moveit_commander.roscpp_initialize(sys.argv)
#    rospy.init_node("hand_control_with_leap_node", anonymous=False)
    robot = moveit_commander.RobotCommander()
    
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    arm_group.set_named_target("home")
    plan_arm = arm_group.go()  
    return arm_group
    

def task_space_control():
    """
    Send goal pose to robot.
    """
    global GOAL_POSE
    human_hand_pos = IMU.hand_pos_calculate()
    GOAL_POSE.position.x = WRIST_POSE.pose.pose.position.x + human_hand_pos[0]
    GOAL_POSE.position.y = WRIST_POSE.pose.pose.position.y + human_hand_pos[1]
    GOAL_POSE.position.z = WRIST_POSE.pose.pose.position.z + human_hand_pos[2]
    print GOAL_POSE
    
        
#        hand_group.set_named_target("handOpen")
#        plan_hand = hand_group.go()  
#        arm_group.set_named_target("home")
#        plan_arm = arm_group.go()  
 
    
    pass



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
    
def odometryCb_ee_link(msg):
	global HAND_POSE
	HAND_POSE = msg
	# print msg.pose.pose

def odometryCb_wrist_1_link(msg):
	global WRIST_POSE
	WRIST_POSE = msg
    # print msg.pose.pose

        

def main():
    try:
		# rospy.init_node('oodometry', anonymous=True) #make node 
#        rospy.sleep(5)
		# joint_space_control(hand_group, arm_group, wrist_2=1.0, wrist_1=1.0)
#        rt_joints_mapping(hand_group, arm_group)
		# sys.exit("done")
		IMU.init_subscribers_and_publishers()
		rospy.Subscriber('/odom_ee_link',Odometry,odometryCb_ee_link)
		rospy.Subscriber('/odom_wrist_1_link',Odometry,odometryCb_wrist_1_link)
		arm_group = movegroup_init()
#        
        
		while not rospy.is_shutdown():
			# print "hand", HAND_POSE.pose.pose.position
			# print "wrist", WRIST_POSE.pose.pose.position
			task_space_control()
#            now = time.time()
#            prev = 0
#            # print "++++", index
#    #        index += 1  This index is to get 5 sensor readings and compute 1 KF estimation. Check pose_node.py in my_human_pkg for more
#            # print "total: ", (now - start)
#            # print "interval:", (now - prev), "calibration_flag:", IMU.calibration_flag
			IMU.update()
			IMU.r.sleep()
#            prev = now
#            Leap.update()
#            Leap.r.sleep()

    except KeyboardInterrupt:
        moveit_commander.roscpp_shutdown()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise


if __name__ == '__main__': main()




