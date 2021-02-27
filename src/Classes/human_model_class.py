#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion as q2e
from math import degrees as r2d


class HumanModel:
    def __init__(self, chest=1.0, upper_arm=1.0, lower_arm=1.0, hand=1.0, rate=100):
        rospy.init_node("human_model_publisher")
        self.r = rospy.Rate(rate)
        self.listener = tf.TransformListener()
        self.trans = Vector3()
        self.rot_q = Quaternion()
        self.rot_e = Vector3()
        self.joint_angle = Float32()
        
    def init_subscribers_and_publishers(self):
        self.pub = rospy.Publisher('/test_joint_angle', Float32, queue_size=1)
        
    def tf_calculator(self, ref_link, target_link):
        try:
            (self.trans, self.rot_q) = self.listener.lookupTransform(ref_link, target_link, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        self.rot_e = q2e([rot_q[0], rot_q[1], rot_q[2], rot_q[3]])
        
    def calculate_joint_angle(self, joint_nr):
        joint_angle_euler_degree = r2d(self.rot_e[joint_nr])
        return joint_angle_euler_degree
        
    def update(self, ref_link="\base_link", target_link="\link_01", joint_nr=0):
        self.tf_calculator(ref_link, target_link)
        self.joint_angle.data = self.calculate_joint_angle(joint_nr)
        self.pub.publish(self.joint_angle)
