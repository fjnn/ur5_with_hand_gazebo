import sys

from math import pi
from math import cos
from math import sin
import numpy as np

from tf.transformations import quaternion_from_matrix as m2q


class DHmatrices:
	def __init__(self):
		""" Initializes 3 rotm and 3 htm and 3 link_vec
		@param number: number of IMUs. 2 for elbow only, 3 for arm only, 4 for include chest"""

		# self.rotm1 = np.zeros((3, 3), dtype = float)
		# self.rotm2 = np.zeros((3, 3), dtype = float)
		# self.rotm3 = np.zeros((3, 3), dtype = float)
		# link_vec1 = np.zeros(3, dtype = float)
		# link_vec2 = np.zeros(3, dtype = float)
		# link_vec3 = np.zeros(3, dtype = float)
		# self.htm1 = np.zeros((4, 4), dtype = float)
		# self.htm2 = np.zeros((4, 4), dtype = float)
		# self.htm3 = np.zeros((4, 4), dtype = float)


		self.rotm = np.zeros((3, 3), dtype = float)
		self.link_vec = np.zeros((3, 1), dtype = float)
		self.htm = np.zeros((4, 4), dtype = float)


		print "Matrices created"


	def angle_to_rotm(self, theta, alpha):
		'''
		https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps
		'''
		self.rotm[0][0] = cos(theta)
		self.rotm[0][1] = -sin(theta) * cos(alpha)
		self.rotm[0][2] = sin(theta) * sin(alpha)
		self.rotm[1][0] = sin(theta)
		self.rotm[1][1] = cos(theta) * cos(alpha)
		self.rotm[1][2] = -cos(theta) * sin(alpha)
		self.rotm[2][1] = sin(alpha)
		self.rotm[2][2] = cos(alpha)
		return self.rotm
	
	
	def link_calculate(self, theta, a, d):
		'''
		https://blog.robotiq.com/how-to-calculate-a-robots-forward-kinematics-in-5-easy-steps
		'''
		self.link_vec[0] = a * cos(theta)
		self.link_vec[1] = a * sin(theta)
		self.link_vec[2] = d
		return self.link_vec
	
		
	def rotm_to_htm(self, rotm, link_vec):
		null_mat = np.concatenate((rotm, link_vec), axis=1)
		print "null_mat_size:", null_mat.shape
		null_vec = np.zeros((1,4), dtype = float)
		print "null_vec_size:", null_vec.shape
		null_vec[0][3] = 1.0
		self.htm = np.concatenate((null_mat, null_vec))
		print "htm:", self.htm
		sys.exit()
		return self.htm
		
		
		
	@staticmethod	
	def matmul(mat1, mat2, mat3):
		return np.matmul(mat1, np.matmul(mat2, mat3))

	@staticmethod
	def htm_to_rotm(htm):
		return htm[np.ix_([0,3],[0,3])]
		
	@staticmethod
	def htm_to_quat(htm):
		rotm = htm[np.ix_([0,3],[0,3])]
		quat = m2q(rotm) # np.array([x, y, z, w])
		return quat
		

