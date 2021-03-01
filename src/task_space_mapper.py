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
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import tf
import math

from Classes.IMU_class_elbow_angle import IMUsubscriber

EE_POSE = Odometry()
WRIST_POSE = Odometry()
GOAL_POSE = Pose()

s = 1.0 # Scale between robot wrist_1_link to ee_link distance and human wrist origin to hand position

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
    arm_group.set_named_target("vertical")
    plan_arm = arm_group.go()  
    return arm_group
    

def task_space_control(arm_group):
    """
    Send goal pose to robot.
    """
    global EE_POSE, GOAL_POSE,s
    robot_init = Pose(Point(-0.175, 0.000, -0.095), Quaternion(0.000, 0.000, -0.707, 0.707))
    GOAL_POSE = IMU.hand_pos_calculate(EE_POSE.pose.pose, robot_init)
    # GOAL_POSE.position.x = WRIST_POSE.pose.pose.position.x + s*IMU.tf_wrist.position.x
    # GOAL_POSE.position.y = WRIST_POSE.pose.pose.position.y + s*IMU.tf_wrist.position.y
    # GOAL_POSE.position.z = WRIST_POSE.pose.pose.position.z + s*IMU.tf_wrist.position.z
    # GOAL_POSE.orientation = IMU.tf_wrist.orientation
    
    print "EE_POSE:", EE_POSE.pose.pose
    print "GOAL_POSE:", GOAL_POSE
    # arm_group.set_pose_target(GOAL_POSE)
    # print "here2"
    # plan_arm = arm_group.go(wait=True) 
    
        
#        hand_group.set_named_target("handOpen")
#        plan_hand = hand_group.go()  
#        arm_group.set_named_target("home")
#        plan_arm = arm_group.go()  

# rostopic echo /odom_ee_link
# header:
  # seq: 340
  # stamp:
    # secs: 146
    # nsecs: 762000000
  # frame_id: "map"
# child_frame_id: "wrist_3_link"
# pose:
  # pose:
    # position:
      # x: 0.0646392808237
      # y: 0.0269655724577
      # z: 1.09875545008
    # orientation:
      # x: 0.0252646798802
      # y: 0.0253592483528
      # z: -0.706673992184
      # w: 0.706633195685
  # covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# twist:
  # twist:
    # linear:
      # x: 0.0025460914695
      # y: 2.04374376156e-05
      # z: -0.000183389034662
    # angular:
      # x: -4.14122316673e-07
      # y: 0.00297111347754
      # z: 0.000309351516229
  # covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
# ---

# rosrun tf tf_echo /ee_link /wrist_1_link
# - Translation: [-0.175, 0.000, -0.095]
# - Rotation: in Quaternion [0.000, 0.000, -0.707, 0.707]
            # in RPY (radian) [0.000, 0.000, -1.571]
            # in RPY (degree) [0.000, 0.002, -89.999]



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
	global EE_POSE
	EE_POSE = msg
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

			# listener = tf.TransformListener()
			arm_group = movegroup_init()
			#        
			
			while not rospy.is_shutdown():
				# (trans,rot) = listener.lookupTransform('/odom_ee_link/pose/pose', '/odom_wrist_1_link/pose/pose', rospy.Time(0))
				if IMU.calibration_flag < 21:
					print "calibration:", IMU.calibration_flag
				else:
					task_space_control(arm_group)
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




