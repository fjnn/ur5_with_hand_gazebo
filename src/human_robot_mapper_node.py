#!/usr/bin/env python

import rospy
from Classes.human_robot_mapper_class import MapperClass
from geometry_msgs.msg import Vector3

if __name__ == "__main__": 
	IMU = MapperClass(START_NODE=True, rate=30)
	while not rospy.is_shutdown():
		MapperClass.update()
		MapperClass.r.sleep()
