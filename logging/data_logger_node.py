#!/usr/bin/env python

"""This is the script that works as a ROS node.
		TODO: make it as gui as you did before.
"""

import rospy, time
import Data.data_logger_module_v2 as data_logger
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from get_model_gazebo_pose import GazeboModel

tool_pose = Pose()
wrist_angles = Vector3()
hand_pose = Pose()
gain = Float32()


def callback_tool_pose(msg):
	global tool_pose_pose
	tool_pose = msg.pose.pose	
	
def callback_human_wrist_angles(msg):
	global wrist_angles
	wrist_angles = msg
	
def callback_hand_pose(msg):
	global hand_pose
	hand_pose = msg
	
def callback_gain(msg):
	global gain
	gain = msg.data
	

if __name__ == "__main__":
	global test_pose, test_gain
	try:
		rospy.init_node('data_logger_node')
		start_time = time.time()
		current_time = rospy.get_time()
		data_logger.enable_logging()
		sub_tool_pose = rospy.Subscriber('/odom_tool0', Odometry, callback_tool_pose) ## Check this if it is the same as wrist_3_link.
		sub_human_wrist_angles = rospy.Subscriber('/wrist_angles', Vector3, callback_human_wrist_angles)
		sub_hand_pose = rospy.Subscriber('/hand_pose', Pose, callback_hand_pose)
		sub_gain = rospy.Subscriber('/gyro_gain', Vector3, callback_gain)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			elapsed_time = time.time() - start_time
			data_logger.log_metrics(elapsed_time, test_pose, test_gain)
			rate.sleep()
	except KeyboardInterrupt:
		data_logger.disable_logging()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
