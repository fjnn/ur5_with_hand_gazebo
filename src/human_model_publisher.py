#!/usr/bin/env python

import rospy
from Classes.human_model_class import HumanModel
from geometry_msgs.msg import Vector3

if __name__ == "__main__": 
    human_model = HumanModel(chest=1.0, upper_arm=1.0, lower_arm=1.0, hand=1.0)
    #TODO: create human model here. The links are useless atm.
    while not rospy.is_shutdown():
        human_model.r.sleep()