#!/usr/bin/env python

import rospy
from Classes.IMU_class_elbow_angle import IMUsubscriber
from geometry_msgs.msg import Vector3

if __name__ == "__main__": 
	IMU = IMUsubscriber(rate=100)
	IMU.init_subscribers_and_publishers()
	while not rospy.is_shutdown():
		IMU.update()
		IMU.r.sleep()
