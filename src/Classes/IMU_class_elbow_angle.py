#!/usr/bin/env python

"""
This is a subscriber. Subscribes the IMU readings and publishes joint angles

"""

# TODO: if body link lengths are given, publish hand position also
# TODO: based on IMU_subscriber_class_v2, log data in data_logger
import sys

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_matrix as q2m
from tf.transformations import euler_from_quaternion as q2e
from tf.transformations import euler_from_matrix as m2e
import Kinematics_with_Quaternions as kinematic
# from my_human_pkg.msg import test_msg

_CALIBRATION_TH = 20
_ROSTIME_START = 0
_HAND_POS_INIT = False
prev = 0
now = 0
hand_link = np.array([0.0, 0.04, 0.0])


class IMUsubscriber:
    def __init__(self, number=2, rate=100):
        """Initializes the IMU data recording node.
        @param number: number of IMUs. 2 for elbow only, 3 for arm only, 4 for include chest"""
        rospy.init_node("imu_subscriber")  # Find a way to get rid of this from here
        self.number = number
        self.r = rospy.Rate(rate)

        self.q_elbow_init = Quaternion(0, 0, 0, 1.0)
        self.q_wrist_init = Quaternion(0, 0, 0, 1.0)

        self.q_elbow = Quaternion(0, 0, 0, 1.0)
        self.q_wrist = Quaternion(0, 0, 0, 1.0)
        self.q_wrist_sensorframe = Quaternion(0, 0, 0, 1.0)
        self.tf_wrist = Pose()

        self.acc_elbow = Vector3()
        self.acc_wrist = Vector3()

        self.gyro_elbow = Vector3()
        self.gyro_wrist = Vector3()

        self.q_wrist_tsm_init = Quaternion(1.0, 1.0, 1.0, 1.0)
        self.q_wrist_tsm = Quaternion(0, 0, 0, 1.0)

        self.hand_pose = Pose()
        self.wrist_angles = Vector3()
        ## this is a bit unnecessary now after wrist_angles_pose
        ## But I want to keep it for consistency to my full arm IMU class
        self.human_joint_imu = JointState()
        self.human_joint_imu.name = ['left_wrist_0', 'left_wrist_1', 'left_wrist_2']
        self.human_joint_imu.position = [0.0, 0.0, 0.0]
        self.calibration_flag = 0
        self.runflag = False
        print self.number, "IMU created"

        # self.my_msg = test_msg()

    def init_subscribers_and_publishers(self):
        self.pub = rospy.Publisher('/human_joint_states', JointState, queue_size=1)
        self.pub_wrist_angles = rospy.Publisher('/wrist_angles', Vector3, queue_size=1)
        self.pub_gyro_wrist = rospy.Publisher('/gyro_wrist', Vector3, queue_size=1)
        self.pub_hand_pose = rospy.Publisher('/hand_pose', Pose, queue_size=1)
#        self.pub_test = rospy.Publisher('/my_test_msg', test_msg, queue_size=10)
#        self.pub_p_hand = rospy.Publisher('/p_hand', Vector3, queue_size=1)
        self.sub_imu_e = rospy.Subscriber('/sensor_l_elbow', Imu, self.cb_imu_elbow)
        self.sub_imu_w = rospy.Subscriber('/sensor_l_wrist', Imu, self.cb_imu_wrist)
        # if self.number==4:
            # self.sub_imu_c = rospy.Subscriber('/sensor_r_wrist', Imu, self.cb_imu_chest)

#        self.log_start_time = rospy.get_time()
        self.runflag = True
#        _ROSTIME_START = rospy.get_time()
        print "IMUs Initialized"

    def update(self):
        # print self.calibration_flag
        self.human_joint_imu.header.stamp = rospy.Time.now()
        if not self.calibration_flag > _CALIBRATION_TH:
            self.calibration_flag = self.calibration_flag + 1
            print "calibrating"
        else:
					self.hand_pos_calculate()
					self.pub.publish(self.human_joint_imu)
					self.pub_hand_pose.publish(self.hand_pose)

					# Clean this part later
					wrist_angles_pose = Vector3(self.wrist_angles[0], self.wrist_angles[1], self.wrist_angles[2])
					self.pub_wrist_angles.publish(wrist_angles_pose)
					self.pub_gyro_wrist.publish(self.gyro_wrist)
					# TODO: here needs update p_hand = hand_link*q_wrist
					# self.p_hand.x = np.degrees(self.human_joint_imu.position[6])  # pitch
					# self.p_hand.y = np.degrees(self.human_joint_imu.position[7])  # yaw
					# self.p_hand.z = np.degrees(self.human_joint_imu.position[8])  # roll
					# self.pub_p_hand.publish(self.p_hand)


    def cb_imu_elbow(self, msg):
        self.elbow_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_elbow_init = kinematic.q_invert(self.elbow_measurement.orientation)
            # print "calibrating elbow"
        self.q_elbow = kinematic.q_multiply(self.q_elbow_init, self.elbow_measurement.orientation)
        # q_elbow_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_shoulder), self.q_elbow)
        self.elbow_angles = q2e(kinematic.q_tf_convert(self.q_elbow), axes='sxyz')
        self.acc_elbow = self.elbow_measurement.linear_acceleration
        self.gyro_elbow = self.elbow_measurement.angular_velocity


    def cb_imu_wrist(self, msg):
        self.wrist_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_wrist_init = kinematic.q_invert(self.wrist_measurement.orientation)
            # print "calibrating wrist"
        self.q_wrist = kinematic.q_multiply(self.q_wrist_init, self.wrist_measurement.orientation)
        self.q_wrist_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_elbow), self.q_wrist)
        self.wrist_angles = q2e(kinematic.q_tf_convert(self.q_wrist_sensorframe), axes='sxyz')
        self.acc_wrist = self.wrist_measurement.linear_acceleration
        self.gyro_wrist = self.wrist_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[0] = self.wrist_angles[0]  # pitch
        self.human_joint_imu.position[1] = self.wrist_angles[1]  # yaw
        self.human_joint_imu.position[2] = self.wrist_angles[2]  # roll

    def hand_pos_calculate2(self, v=hand_link):
			"""
			NOTE: Use this when you calibrate hand in BENT pose
			Calculate current hand_pose (self.tf_wrist)
			@param v=hand_link default
			"""
			global _HAND_POS_INIT, v_rotated_init
			hand_pose = Pose()
			if _HAND_POS_INIT == False:
					v_rotated_init = np.array([0.0, 0.0, 0.0])
					print "Move the hand to the BENT pose. Press Enter..."
					dummy_input = raw_input()
					self.q_wrist_tsm_init = kinematic.q_invert(self.q_wrist_sensorframe)
					v_rotated_init = kinematic.q_rotate(self.q_wrist_sensorframe, hand_link)
					# print "calibration:", self.calibration_flag, "self.q_wrist_tsm_init:", self.q_wrist_tsm_init
					print "v_rotated_init:", v_rotated_init
					_HAND_POS_INIT = True
			else:
					# Init origin poses and the quaternion will be this
					self.q_wrist_tsm = kinematic.q_multiply(self.q_wrist_tsm_init, self.q_wrist_sensorframe)
					v_rotated = kinematic.q_rotate(self.q_wrist_sensorframe, hand_link)
					# Assign the displacement
					self.tf_wrist.position.x = v_rotated[0]-v_rotated_init[0]
					self.tf_wrist.position.y = v_rotated[1]-v_rotated_init[1]
					self.tf_wrist.position.z = v_rotated[2]-v_rotated_init[2]

					self.hand_pose.position.x = v_rotated[0]
					self.hand_pose.position.y = v_rotated[1]
					self.hand_pose.position.z = v_rotated[2]
					self.tf_wrist.orientation = self.q_wrist_tsm
					# self.tf_wrist.orientation = kinematic.q_multiply(robot_init.orientation, self.q_wrist_tsm)
					self.hand_pose.orientation = self.q_wrist_tsm


    def hand_pos_calculate(self, v=hand_link):
				"""
				NOTE: Use this when you calibrate hand in HORIZONTAL pose
				Calculate current hand_pose
				@param v=hand_link default
				"""
				global _HAND_POS_INIT
				v_rotated = kinematic.q_rotate(self.q_wrist_sensorframe, hand_link)
				# print "Wrist_angles:", self.q_wrist_sensorframe
				# print "calibration:", self.calibration_flag, "self.q_wrist_tsm_init:", self.q_wrist_tsm_init
				print "v_rotated_init:", v_rotated
				if _HAND_POS_INIT == False:
					print "Cont?"
					dummy_input = raw_input()
					_HAND_POS_INIT = True

				# Calculate pose
				self.hand_pose.position.x = v_rotated[0]
				self.hand_pose.position.y = v_rotated[1]
				self.hand_pose.position.z = v_rotated[2]
				self.hand_pose.orientation = self.q_wrist_tsm
