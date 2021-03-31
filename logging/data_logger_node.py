#!/usr/bin/env python

"""This is the script that works as a ROS node.
		TODO: make it as gui as you did before.
"""

import rospy, time
import Data.data_logger_module_v2 as data_logger
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8

test_pose = Pose()
test_gain = Int8()


def callback_pose(msg):
	global test_pose
	test_pose = msg
	# print "test_pose:", test_pose
	
def callback_gain(msg):
	global test_gain
	test_gain = msg.data
	# print "test_gain:", test_gain
	

if __name__ == "__main__":
	global test_pose, test_gain
	try:
		rospy.init_node('data_logger_node')
		# start_time = rospy.get_time()
		start_time = time.time()
		current_time = rospy.get_time()
		data_logger.enable_logging()
		sub_testpose = rospy.Subscriber('/Tee_goal_pose', Pose, callback_pose)
		sub_testgain = rospy.Subscriber('/selector', Int8, callback_gain)
		rate = rospy.Rate(10)
		while not rospy.is_shutdown():
			elapsed_time = time.time() - start_time
			# elapsed_time = current_time - start_time
			current_time = rospy.get_time()
			data_logger.log_metrics(elapsed_time, test_pose, test_gain)
			rate.sleep()
	except KeyboardInterrupt:
		data_logger.disable_logging()
        rospy.signal_shutdown("KeyboardInterrupt")
        raise
