#!/usr/bin/env python

"""This is the script that works as a ROS node.
		TODO: make it as gui as you did before.
"""

import rospy
import Data.data_logger_module_v2 as data_logger


if __name__ == "__main__":
		rospy.init_node('data_logger_node')
    data_logger.enable_logging()
		# subscribers here...
		rate = rospy.Rate(10)
    while not rospy.shutdown:
        rate.sleep()
