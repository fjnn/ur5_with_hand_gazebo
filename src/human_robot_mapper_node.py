#!/usr/bin/env python

import rospy
import sys, time
from Classes.human_robot_mapper_class_v2 import MapperClass
from geometry_msgs.msg import Vector3

def main():
	mapper = MapperClass(START_NODE=True, rate=30)
	mapper.init_subscribers_and_publishers()
	start_time = rospy.Time.now()
	while not rospy.is_shutdown():
		current_time = rospy.Time.now()
		elapsed_time = current_time-start_time
		# print "Elapsed time iksolver:", elapsed_time
		mapper.update(elapsed_time)
		mapper.r.sleep()

if __name__ == "__main__": main ()
