#!/usr/bin/env python3.7
import rospy
import math
import time
import numpy as np
from TOMO_final.msg import position_blisters  
from pyquaternion import Quaternion
import sys
import os
from TOMO_final.msg import cam_to_object
import scripts.transformation as tf
# from tf.transformations import quaternion_matrix
sys.path.append("..")
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3

# from software.Blisster_De.densefusion.tools import size_estimation_for_hand   
pub = rospy.Publisher('/position_orientation_blisters',position_blisters,queue_size=10)
rospy.init_node("image_process")

# sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') # append back in order to import rospy


class matrix(object):
	def __init__(self,pub,mat):

		self.H_cam_to_object = []
		for i in range (len(mat)):
			self.H_cam_to_object.append(mat[i])
		self.rate = rospy.Rate(10) 

	def publish_data(self):
		H  =        [[0.000156331819098,  -0.008260427459506,   0.005633906161319,   1.107856016625151*10**(-3)],
				[-0.009998751581336,  -0.000116213242994,   0.000107057509094,   0.345533044416339*10**(-3)],
				[-0.000022960628060,  -0.005634876464872,  -0.008261212989576,   4.352870519336399*10**(-3)],
				[                0,                   0,                   0,   0.010000000000000]]
			
		self.H_base_to_cam = np.dot(100,H)

		#matrix from base to object
		self.H_base_to_object = [] 

		#Quaternion
		self.q8d = [] 

		#position of blisters
		self.po_x = [] 
		self.po_y = []
		self.po_z = []

		#orientation of blisters
		self.ori_x = []
		self.ori_y = []
		self.ori_z = []
		self.ori_w = []

		for i in range (len(self.H_cam_to_object)):
			self.H_base_to_object.append(np.dot(self.H_base_to_cam,self.H_cam_to_object[i]))

		for i in range (len(self.H_base_to_object)):
			self.q8d.append(Quaternion(matrix=self.H_base_to_object[i])) 

		for i in range (len(self.H_base_to_object)):
			self.po_x.append(self.H_base_to_object[i][0][3])
			self.po_y.append(self.H_base_to_object[i][1][3])
			self.po_z.append(self.H_base_to_object[i][2][3])

		print('po_x: ', self.po_x)
		print('po_y: ', self.po_y)
		print('po_z: ', self.po_z)
		
		for i in range (len(self.q8d)):
			self.ori_x.append(round(self.q8d[i][1],2))
			self.ori_y.append(round(self.q8d[i][2],2))
			self.ori_z.append(round(self.q8d[i][3],2))
			self.ori_w.append(round(self.q8d[i][0],2))

		print(self.ori_w,self.ori_x,self.ori_y,self.ori_z )

		self._pub = pub
		blisters = position_blisters()
		blisters.po_x = self.po_x #base to object 
		blisters.po_y = self.po_y
		blisters.po_z = self.po_z

		blisters.ori_x = self.ori_x
		blisters.ori_y = self.ori_y
		blisters.ori_z = self.ori_z
		blisters.ori_w = self.ori_w

		self._pub.publish(blisters)
		self.rate.sleep()
def callback(data):
	global _matrix
	_matrix = []
	x_t_cam = data.x_t_cam
	y_t_cam = data.y_t_cam
	z_t_cam = data.z_t_cam

	x_r_cam = data.x_r_cam
	y_r_cam = data.y_r_cam
	z_r_cam = data.z_r_cam
	w_r_cam = data.w_r_cam

	for i in range (len(x_r_cam)):
		_matrix.append(tf.quaternion_matrix([w_r_cam[i], x_r_cam[i], y_r_cam[i], z_r_cam[i]]))
	for i in range (len(x_r_cam)):
		_matrix[i][0][3] = x_t_cam[i]
		_matrix[i][1][3] = y_t_cam[i]
		_matrix[i][2][3] = z_t_cam[i]
	return _matrix   

if __name__ == '__main__':
	while(True):
		rospy.Subscriber('/cam_to_object', cam_to_object, callback)
		time.sleep(1)
		#    mat = size_estimation_for_hand.check_one_frame()   
		blisters = matrix(pub,_matrix)
		try:
			blisters.publish_data()
			pass

		except KeyboardInterrupt:

			break
