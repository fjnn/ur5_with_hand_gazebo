# -*- coding: utf-8 -*-

"""
This is a subscriber. Subscribes the Leap motion readings and publishes necessary attributes

"""

# imports
import rospy
import math
from leap_motion.msg import Human
from std_msgs.msg import Float32


class LeapSubscriber:
    def __init__(self, hand="left", rate=100):
        """Initializes the IMU data recording node.
        TODO: give option to chose hand left or right"""
#        rospy.init_node("leap_subscriber")  # Find a way to get rid of this from here
        self.r = rospy.Rate(rate)
        self.grab_strength = Float32()
        print "Leap created"

    def init_subscribers_and_publishers(self):
        self.pub = rospy.Publisher('/grab_strength', Float32, queue_size=1)
        self.sub_gripper = rospy.Subscriber('/leap_motion/leap_filtered', Human, self.callback_leap)
        print "Leap Initialized"
        
    def callback_leap(self, hand_msg):  # callback for gripper message
        self.hand_msg = hand_msg
        if hand_msg.left_hand.is_present:
            self.grab_strength.data = self.hand_msg.left_hand.grab_strength
#            print "grab_strength:", self.grab_strenght
#        else:
#            print "no hand detected"
    
    def update(self):
        self.pub.publish(self.grab_strength)
