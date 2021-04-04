#! /usr/bin/env python

"""
This is a node to publish tool0 transformation

Gazebo odometry publisher cannot be linked to tool0 but it connects to wrist_3_link.
		The transformation from wrist_3_link to tool 0 is fixed
			- Translation: [0.000, 0.082, 0.000]
			- Rotation: in Quaternion [-0.707, 0.000, 0.000, 0.707]	
If this node doesn't do what I want, I will use human_robot_mapper_class_v2.py/correct_tool0() method
"""

import rospy

import math
import tf2_ros
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('tf2_tool0_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'tool0', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

        print "Translation:", trans.transform.translation
        print "Rotation:", trans.transform.rotation
        msg = geometry_msgs.msg.Twist()

        rate.sleep()

'''
base_link to tool0 at all zero joints
--------------------------------------
Translation: x: 0.816109976384
y: 0.163138372342
z: 0.0553127183891
Rotation: x: 0.00385217367038
y: 0.707094729728
z: 0.707097503324
w: -0.00391468278267
'''