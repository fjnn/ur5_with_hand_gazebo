#!/usr/bin/env python

"""
This is a subscriber. Subscribes the IMU readings and publishes joint angles

"""

# TODO: if body link lengths are given, publish hand position also
# TODO: based on IMU_subscriber_class_v2, log data in data_logger

import rospy
import numpy as np
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_matrix as q2m
from tf.transformations import euler_from_quaternion as q2e
from tf.transformations import euler_from_matrix as m2e
import Kinematics_with_Quaternions as kinematic
# from my_human_pkg.msg import test_msg

_CALIBRATION_TH = 20
_ROSTIME_START = 0
prev = 0
now = 0


class IMUsubscriber:
    def __init__(self, number=3, rate=100):
        """Initializes the IMU data recording node.
        @param number: number of IMUs. 3 for only arm, 4 for include chest"""
        rospy.init_node("imu_subscriber")  # Find a way to get rid of this from here
        self.number = number
        self.r = rospy.Rate(rate)

        self.q_chest_init = Quaternion(0, 0, 0, 1.0)
        self.q_shoulder_init = Quaternion(0, 0, 0, 1.0)
        self.q_elbow_init = Quaternion(0, 0, 0, 1.0)
        self.q_wrist_init = Quaternion(0, 0, 0, 1.0)

        self.q_chest = Quaternion(0, 0, 0, 1.0)
        self.q_shoulder = Quaternion(0, 0, 0, 1.0)
        self.q_elbow = Quaternion(0, 0, 0, 1.0)
        self.q_wrist = Quaternion(0, 0, 0, 1.0)

        self.acc_chest = Vector3()
        self.acc_shoulder = Vector3()
        self.acc_elbow = Vector3()
        self.acc_wrist = Vector3()

        self.gyro_chest = Vector3()
        self.gyro_shoulder = Vector3()
        self.gyro_elbow = Vector3()
        self.gyro_wrist = Vector3()

        self.p_hand = Vector3()
        self.human_joint_imu = JointState()
        self.human_joint_imu.name = ['left_shoulder_2', 'left_shoulder_0', 'left_shoulder_1', 'left_elbow_2', 'left_elbow_0', 'left_elbow_1', 'left_wrist_2', 'left_wrist_0', 'left_wrist_1']
        self.human_joint_imu.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.calibration_flag = 0
        self.runflag = False
        print self.number, "IMU created"

        # self.my_msg = test_msg()

    def init_subscribers_and_publishers(self):
        self.pub = rospy.Publisher('/human_joint_states', JointState, queue_size=1)
#        self.pub_test = rospy.Publisher('/my_test_msg', test_msg, queue_size=10)
#        self.pub_p_hand = rospy.Publisher('/p_hand', Vector3, queue_size=1)
        self.sub_imu_s = rospy.Subscriber('/sensor_l_shoulder', Imu, self.cb_imu_shoulder)
        self.sub_imu_e = rospy.Subscriber('/sensor_l_elbow', Imu, self.cb_imu_elbow)
        self.sub_imu_w = rospy.Subscriber('/sensor_l_wrist', Imu, self.cb_imu_wrist)
        if self.number==4:
            self.sub_imu_c = rospy.Subscriber('/sensor_r_wrist', Imu, self.cb_imu_chest)
        
#        self.log_start_time = rospy.get_time()
        self.runflag = True
#        _ROSTIME_START = rospy.get_time()
        print "IMUs Initialized"

    def update(self):
        # print self.calibration_flag
        self.human_joint_imu.header.stamp = rospy.Time.now()
        if not self.calibration_flag > _CALIBRATION_TH:
            self.calibration_flag = self.calibration_flag + 1
        self.pub.publish(self.human_joint_imu)
        # TODO: here needs update p_hand = hand_link*q_wrist
        # self.p_hand.x = np.degrees(self.human_joint_imu.position[6])  # pitch
        # self.p_hand.y = np.degrees(self.human_joint_imu.position[7])  # yaw
        # self.p_hand.z = np.degrees(self.human_joint_imu.position[8])  # roll
        # self.pub_p_hand.publish(self.p_hand)

    def update2(self, human_joint_kf):
        '''
        Just to check if KF works fine on Rviz. Details are in the func in body.calculate_joint_angles().
        '''
        global now, prev
        # print self.calibration_flag
        self.joint_kf.position = human_joint_kf.position
        self.joint_kf.header.stamp = rospy.Time.now()
        self.calibration_flag = self.calibration_flag + 1

        self.pub.publish(self.joint_kf)
        self.pub_test.publish(self.my_msg)

    def cb_imu_chest(self, msg):
        self.chest_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_chest_init = kinematic.q_invert(self.chest_measurement.orientation)
            # print "calibrating chest"
        self.q_chest = kinematic.q_multiply(self.q_chest_init, self.chest_measurement.orientation)
        self.acc_chest = self.chest_measurement.linear_acceleration
        self.gyro_chest = self.chest_measurement.angular_velocity
        self.chest_angles = q2e(kinematic.q_tf_convert(self.q_chest), axes='sxyz')

    def cb_imu_shoulder(self, msg):
        self.shoulder_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_shoulder_init = kinematic.q_invert(self.shoulder_measurement.orientation)
            # print "calibrating shoulder"
        self.q_shoulder = kinematic.q_multiply(self.q_shoulder_init, self.shoulder_measurement.orientation)
        # print "q_measured: {0} \n q_init:{1}".format(self.shoulder_measurement.orientation, self.q_shoulder_init)
        self.shoulder_angles = q2e(kinematic.q_tf_convert(self.q_shoulder), axes='sxyz')
        self.acc_shoulder = self.shoulder_measurement.linear_acceleration
        self.gyro_shoulder = self.shoulder_measurement.angular_velocity
        # q_shoulder_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_chest), self.q_shoulder)
        # self.shoulder_angles = q2e(kinematic.q_tf_convert(q_shoulder_sensorframe), axes='sxyz')
        # Update joint angles
        self.human_joint_imu.position[0] = self.shoulder_angles[0]  # pitch
        self.human_joint_imu.position[1] = self.shoulder_angles[1]  # yaw
        self.human_joint_imu.position[2] = self.shoulder_angles[2]  # roll

    def cb_imu_elbow(self, msg):
        self.elbow_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_elbow_init = kinematic.q_invert(self.elbow_measurement.orientation)
            # print "calibrating elbow"
        self.q_elbow = kinematic.q_multiply(self.q_elbow_init, self.elbow_measurement.orientation)
        q_elbow_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_shoulder), self.q_elbow)
        self.elbow_angles = q2e(kinematic.q_tf_convert(q_elbow_sensorframe), axes='sxyz')
        self.acc_elbow = self.elbow_measurement.linear_acceleration
        self.gyro_elbow = self.elbow_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[3] = self.elbow_angles[0]  # pitch
        self.human_joint_imu.position[4] = self.elbow_angles[1]  # yaw
        self.human_joint_imu.position[5] = self.elbow_angles[2]  # roll

    def cb_imu_wrist(self, msg):
        self.wrist_measurement = msg
        while self.calibration_flag < _CALIBRATION_TH:
            self.q_wrist_init = kinematic.q_invert(self.wrist_measurement.orientation)
            # print "calibrating wrist"
        self.q_wrist = kinematic.q_multiply(self.q_wrist_init, self.wrist_measurement.orientation)
        q_wrist_sensorframe = kinematic.q_multiply(kinematic.q_invert(self.q_elbow), self.q_wrist)
        self.wrist_angles = q2e(kinematic.q_tf_convert(q_wrist_sensorframe), axes='sxyz')
        self.acc_wrist = self.wrist_measurement.linear_acceleration
        self.gyro_wrist = self.wrist_measurement.angular_velocity
        # Update joint angles
        self.human_joint_imu.position[6] = self.wrist_angles[0]  # pitch
        self.human_joint_imu.position[7] = self.wrist_angles[1]  # yaw
        self.human_joint_imu.position[8] = self.wrist_angles[2]  # roll
        
#    def calculate_hand_pos(self, ref_link):
#        self.
