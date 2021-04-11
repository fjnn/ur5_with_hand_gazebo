#! /usr/bin/env python

"""
This is a node subscribes joint states, drives the robot. WITHOUT MOVEIT.
"""

import sys
import time
import copy
from math import pi
from math import radians as d2r

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Vector3

import actionlib
import control_msgs.msg as cm
import trajectory_msgs.msg as tm

JOINT_NAMES = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] # gazebo model has this interesting order of joints
               
# home = [0.0, -1.6571, -1.5880, 0.0, 1.5880, -0.05]
home = [pi/2, -pi/2, 0.0, pi, -pi/2, 0.0]
pivot = [0.2649, -1.38946, -2.47837, -0.802851, 1.57, 0.266337]
current_joint_values = [pi/2, -pi/2, 0.0, 0.0, 0.0, 0.0]

client = None

desired_URwrist_joints = Vector3()



def move_single_joint(current):
    global joints_pos
    g = cm.FollowJointTrajectoryGoal()
    g.trajectory = tm.JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            tm.JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            tm.JointTrajectoryPoint(positions=current, velocities=[0]*6, time_from_start=rospy.Duration(0.4))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
        
def move_home():
    global joints_pos
    g = cm.FollowJointTrajectoryGoal()
    g.trajectory = tm.JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            tm.JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            tm.JointTrajectoryPoint(positions=home, velocities=[0]*6, time_from_start=rospy.Duration(2.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise

        
def cb_joint_goal(msg):
    print "msg:", msg.position
    desired_URwrist_joints.x = msg.position[3] 
    desired_URwrist_joints.y = msg.position[4] 
    desired_URwrist_joints.z = msg.position[5] 
    print "desired_URwrist_joints", desired_URwrist_joints

def main():
    global client, current_joint_values
    
    try:
        rospy.init_node("send_joint_commands", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', cm.FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        print "Press Enter to proceed:"
        dummy_input = raw_input()
        move_home()
        dummy_input = raw_input("home?")

        sub_tf_listener = rospy.Subscriber('/mapper_joints', JointState, cb_joint_goal)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            current_joint_values[3] = desired_URwrist_joints.x + home[3]
            current_joint_values[4] = desired_URwrist_joints.y + home[4]
            current_joint_values[5] = desired_URwrist_joints.z + home[5]
            print current_joint_values
            move_single_joint(current_joint_values)
            rate.sleep()

    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()



