#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Calibration procedure in Xsens.

"""
import numpy as np
import pyquaternion as pq
from math import pi as pi
from math import sqrt
from math import acos
from math import degrees as r2d
from geometry_msgs.msg import Quaternion
from tf.transformations import euler_from_quaternion as q2e
from tf.transformations import quaternion_from_matrix as m2q
from tf.transformations import rotation_matrix
from tf.transformations import euler_matrix
from tf.transformations import concatenate_matrices
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_conjugate

import scipy.integrate as integrate
import matplotlib.pyplot as plt

import Classes.Kinematics_with_Quaternions as kinematic
# import Classes.body as body
# from Classes.IMU_subscriber_class_v2 import IMUsubscriber
#
# test = IMUsubscriber()


s = np.array([0, -6, 0])
sq = np.array([0, -6, 0, 0])


qz = pq.Quaternion(angle = np.pi/6, axis=[0, 0, 1])
qy = pq.Quaternion(angle = np.pi/6, axis=[0, 1, 0])

Rz = np.array([ [0.8660254, -0.5000000,  0.0000000],[ 0.5000000,  0.8660254,  0.0000000],[ 0.0000000,  0.0000000,  1.0000000 ]])
Ry = np.array([ [0.8660254,  0.0000000,  0.5000000],[ 0.0000000,  1.0000000,  0.0000000],[ -0.5000000,  0.0000000,  0.8660254 ]])

Rz_new = rotation_matrix(pi/6, (0, 0, 1))
Ry_new = rotation_matrix(pi/6, (0, 1, 0))
qRz = m2q(Rz_new)
qRy = m2q(Ry_new)
# qRzy = quaternion_multiply(qRy, qRz)


Rzy = Ry.dot(Rz)
sy = Rzy.dot(s)

sq1 = qz.rotate([0, 0, 6])
# sq2 = qy.rotate(qz).rotate(s)
# q12 = qy.rotate(qz)

# print qz
# print s

q1 = np.array([0.966, 0, 0, 0.259])
v1 = np.array([0, 0, 6])


# def q_rotate(q, v):
#     ''''
#     Computes v_rotated = qvq* fastly
#     @params q: rotating quaternion represented as numpy array [q0, q1, q2, q3] -> [w, x, y, z]
#     @params v: to be rotated vector. Represented as numpy array. Expressed as pure quaternion. [v0, v1, v2] -> [x, y, z]
#     '''
#     v_rotated = np.array([v[0]*(q[0]**2 + q[1]**2 - q[2]**2 - q[3]**2) + 2*v[1]*(q[1]*q[2]-q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[1]*q[3]),
#                           2*v[0]*(q[0]*q[3] + q[1]*q[2]) + v[1]*(q[0]**2 - q[1]**2 + q[2]**2 - q[3]**2) + 2*v[2]*(q[2]*q[3] - q[0]*q[1]),
#                           2*v[0]*(q[1]*q[3] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[2]*q[3]) + v[2]*(q[0]**2 - q[1]**2 - q[2]**2 + q[3]**2)])
#
#     # v2 = np.concatenate([[0], v1], axis=0)
#     # q2 = quaternion_multiply(v2, quaternion_conjugate(q1))
#     # q2 = quaternion_multiply(q1, q2)
#     return v_rotated


# test integration
t = np.array([x for x in range(0, 6)])
v = np.array([x*2 for x in t])
# plt.plot(t, v)
# plt.show()
v[-1] = 0
result_w = 0
for i in t:
    v_window = [v[i-1], v[i]]
    result_w = integrate.simps(v_window) + result_w
result = integrate.simps(v)


# test 3D integration
# dt = 0.1
# a1 = np.array([0, 0, 0])
# a2 = np.array([3, 2, 0])
# a = np.subtract(a2, a1)
# a_integrated = integrate.simps(a, dx=dt)
# print a_integrated


# test quaternion inverse
# measurement_quat = pq.Quaternion(angle=np.pi/3, axis=[1, 0, 0])
# print measurement_quat.x
# # init_quat = measurement_quat.inverse
# init_quat = kinematic.q_invert(measurement_quat)
# measurement_quat = pq.Quaternion(angle=2*np.pi/9, axis=[1, 0, 0])
# current_quat = measurement_quat * init_quat
# print current_quat

# test quaternion_convert
# q1 = Quaternion(0,0,0,1.0)
# q2 = Quaternion(-0.181083, 0, 0, 0.9834678)
# q3 = kinematic.q_tf_convert(kinematic.q_multiply(q1, q2))
# print kinematic.q_tf_convert(q1)
# q3 = kinematic.q_multiply(kinematic.q_tf_convert(q1), kinematic.q_tf_convert(q2))
# print r2d(q2e(q3)[0])
# q2e([1.0, 0.0, 0.0, 0.0], axes='sxyz')


# List append test:
# l_hand = np.array([0.06, 0.0, 0.0])
# l_la = np.array([0.3, 0.0, 0.0])
# l_ua = np.array([0.3, 0.0, 0.0])
# l_chest = np.array([0.0, 0.0, 0.25])

# Quaternion rotation order
# q_z90 = Quaternion(0, 0, -0.7071068, 0.7071068)
# q_x90 = Quaternion(-0.7071068, 0, 0, 0.7071068)
# q_gs_init = kinematic.q_multiply(q_z90, q_x90)
# print q_gs_init

# numpy multiply
# a = np.array([[2,3],[4,5]])
# b = np.array([1,2])
# print a.dot(b)

# from numpy vector to quaternion
a = np.array([1, 0, 0])  # dönünce [0, 1, 0] bekliyorum
a_quat = Quaternion(a[0], a[1], a[2], 0)
b = np.array([0.4, 0.0, 0.0])
rot_b_quat = Quaternion(0.0, 0.7071068, 0.0, 0.7071068)
q_z90 = Quaternion(0, 0, -0.7071068, 0.7071068)
# rotated = kinematic.q_multiply(q_z90, a_quat)
rotated = kinematic.q_rotate(rot_b_quat, b)
print rotated
rotated_norm = kinematic.q_norm(rotated)
print rotated_norm
# print kinematic.q_scale(rotated, 2)

# quaternion integration for angular_velocity
# q_sensor = Quaternion(0, 0, 0, 1)
# p1 = [0, 1, 0]
# # omega = [pi/2, pi/4, pi/12]
# omega = [pi/5, 0, 0]
# theta = [pi/2, 3*pi/4, pi/4]
# omega_quat = Quaternion(omega[0], omega[1], omega[2], 0)
# omega_quat_norm = kinematic.q_norm(omega_quat)
# q_dot = kinematic.q_scale(kinematic.q_multiply(q_sensor, omega_quat), 0.5)
# # q_dot = kinematic.q_multiply(q_sensor, omega_quat_norm)
# print "q_dot:", q_dot
#
# q_sensor.x += integrate.simps([q_dot.x, q_dot.x], dx=5)
# q_sensor.y += integrate.simps([q_dot.y, q_dot.y], dx=5)
# q_sensor.z += integrate.simps([q_dot.z, q_dot.z], dx=5)
#
# print "q_sensor_mag:", kinematic.q_magnitude(q_sensor)
#
# q_sensor_norm = kinematic.q_norm(q_sensor)  # I am not sure if it is required - according to that, yes: https://stackoverflow.com/questions/46908345/integrate-angular-velocity-as-quaternion-rotation
# p2 = kinematic.q_rotate(q_sensor_norm, p1)
# print p2
# # sağlamasını yapamadım. tam olarak neden 1/2*q*w yaptığımızı bi anla.
# # ya da vazgeçtim. gyro integration olaylarına hiç girmeyeceğim. orientation2u direkt xsens'in orientation'u olarak kullanacağım. Yine de standford şeysini oku lecture-10

# body update
# body = []
# l_hand = 1 * np.array([1.0, 0.0, 0.0])
# l_la = 2 * np.array([1.0, 0.0, 0.0])
# l_ua = 3 * np.array([1.0, 0.0, 0.0])
# l_chest = 4 * np.array([0.0, 0.0, 1.0])
#
# body.append(l_chest)
# body.append(l_ua)
# body.append(l_la)
# body.append(l_hand)
#
# print body
# for body_link in range(1, 4):
#     body[body_link-1] = body[body_link]
#
# print body

# joint_pos = np.array([[1, 0, 0], [0.6, 0.8, 0], [20, -52, 0], [20, -57, 0]])
# # kinematic.find_angle(joint_pos[0], joint_pos[1])
# angles, q_result = kinematic.find_angle(joint_pos[0], joint_pos[1])
# print angles
# print kinematic.q_rotate(q_result, joint_pos[0])  # sağlama
# # body.calculate_joint_angles(joint_pos)

# ZeroDivisionError test
# a = 0
# b = 2
# # if not (a == 0 or b == 0):
# #     result = 5.0/a
# # else:
# #     print "zero division"
# #     result = 3
# # print result
#
# try:
#     result = 5.0/a
# except ZeroDivisionError:
#     print "zero division"
#     result = 3
# print result
