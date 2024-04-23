#!/usr/bin/env python

#############################################################
# This file calculates the cameras extrinsic parameters and 
# saves their results into "camera_calibration_results"
#############################################################

import numpy as np
import os
# from cv2 import cv2
import cv2
import random
import yaml
from tqdm import tqdm
from time import sleep
import matplotlib.pyplot as plt
from scipy.optimize import least_squares

control_point_path = os.path.dirname(os.getcwd()) + r'/control_points/'
camera_intrinsic_path =os.path.dirname(os.getcwd())+ r'/Config/camera_calibration_results/Intrinsic/'
camera_calibration_results_path = os.path.dirname(os.getcwd()) + r'/Config/camera_calibration_results/Extrinsic/'

camera_extrinsic_filename = ['camera1.yaml', 'camera2.yaml']

def last_4chars(x):
    return(x[-8:])
control_point_filename = os.listdir(control_point_path)
camera_intrinsic_filename = os.listdir(camera_intrinsic_path)

control_point_filename_sorted = sorted(control_point_filename, key = last_4chars)
camera_intrinsic_filename_sorted = sorted(camera_intrinsic_filename, key = last_4chars)
#print(camera_intrinsic_filename)
#print(camera_intrinsic_filename_sorted)
pbar = tqdm(total=2)
def func(bet, truth, obs):
    numPoints = len(truth)
    beta1 = bet[0]
    beta2 = bet[1]
    beta3 = bet[2]
    
    r0 = np.array([beta1, beta2,0])
    R = np.array([[np.cos(beta3), -np.sin(beta3),0],
                    [np.sin(beta3), np.cos(beta3),0],
					[0            ,  0           ,1]])
    r = np.zeros(numPoints)
    for i in range(numPoints):
        r[i] = np.linalg.norm(truth[i] - (r0 + R.dot(obs[i]))) 
    
    return r

for i in range(len(camera_intrinsic_filename_sorted)):

	camera_file = camera_intrinsic_path + camera_intrinsic_filename_sorted[i]
	control_point_file = control_point_path + control_point_filename_sorted[i]
	#print(i)
	with open(camera_file) as file:
		# yaml_data = yaml.full_load(file)
		yaml_data = yaml.load(file)
		file.close()

	K = np.array(yaml_data['camera_matrix']['data']).reshape((3,3))
	d = np.array(yaml_data['distortion_coefficients']['data'])
	control_point_data = np.unique(np.loadtxt(control_point_file, delimiter=','), axis=0)
	#print(K)
	object_points = control_point_data[:,:3]
	image_points = control_point_data[:,-2:]
	print(object_points)
	print(image_points)

	_, rVec, tVec, inl = cv2.solvePnPRansac(objectPoints=object_points, imagePoints=image_points, cameraMatrix=K, distCoeffs=d, reprojectionError=0.5)
	print("inl",inl)
	#print("inl is empty!")
    

	#object_points = np.delete(object_points, inl, axis=0)
	#image_points = np.delete(image_points, inl, axis=0)

	# print(np.vstack((object_points, image_points)))

	# print(object_points)
	# print(image_points)

	# _, rVec, tVec = cv2.solvePnP(object_points, image_points, K, d, flags=cv2.SOLVEPNP_ITERATIVE)
	# _, rVec, tVec, inl = cv2.solvePnPRansac(objectPoints=object_points, imagePoints=image_points, cameraMatrix=K, distCoeffs=d)
	
	R, _ = cv2.Rodrigues(rVec)
	#print(R, tVec)




	#############################
	### Least Sqaure Added    ###
	###                       ###
	###        START          ###
	#############################
	
	Rz = np.diag((-1, -1, 1))
	R_ = R.dot(Rz)
	X0 = -np.dot(R.T, tVec)
	H = np.linalg.pinv(np.dot(K,R_))
	image_points = np.insert(image_points, 2, 1, axis=1)
	image_points_= np.zeros((1,3))
	for j in range(len(image_points)):
		point = np.array(image_points[j]).reshape((3,1))

		H1 = H.dot(point)
		lamb = (X0[-1]+0.1) / H1[-1]
		point = X0 + lamb * H1
	
		
		image_points_ = np.append(image_points_, point.T,axis = 0)
	image_points_ = image_points_[1:,:]
	
	
	plt.subplot(121)
	plt.plot(object_points[:,0], object_points[:,1], 'kp', label='Ground Truth')
	plt.plot(image_points_[:,0], image_points_[:,1], 'rp', label='PNPRansac')
	plt.legend(bbox_to_anchor=(0., 1.02, 1., .102), loc='lower left',ncol=2, mode="expand", borderaxespad=0.)
	'''
	#alph = np.arccos(-(R_[1,0]/R_[2,0]))
	#print (R_[2,0],R_[1,0],alph)
	beta = np.array([0,0,0.037])
	ans = least_squares(func, beta, method='lm', args=(object_points, image_points_,))
	beta = ans['x']
	


	X0_l = np.array([beta[0], beta[1],0])
	R_l = np.array([[np.cos(beta[2]), -np.sin(beta[2]),0],
                [np.sin(beta[2]), np.cos(beta[2]),0],
				[0,0,1]])
	print(X0_l, R_l,R)
	image_points_l = np.zeros((1,3))
	for j in range(len(image_points_)):
		image_points_[j]= X0_l + R_l.dot(image_points_[j])
	
	plt.plot(image_points_[:,0], image_points_[:,1], 'bp', label='LeastSquare')
    '''
	plt.show()











	#############################
	### END  ####
	#############################


	# Rz = np.diag((-1, -1, 1))

	# R = R.dot(Rz)
    
	camera_extrinsics = dict(rotation_matrix = R.tolist(), location = (-np.dot(R.T, tVec)).tolist(), K = K.tolist())
	#camera_extrinsics = dict(rotation_matrix = R.tolist(), location = (-np.dot(R.T, tVec)).tolist(), K = K.tolist())
	# camera_extrinsics = dict(rotation_matrix = R.tolist(), location = (-np.dot(R.T, tVec)).tolist(), K = K.tolist(), inliers=len(inl))
	# camera_extrinsics = dict(rotation_matrix = R.tolist(), location = tVec.tolist(), K = K.tolist(), inliers=len(inl))


	object_points = []
	image_points = []
	with open(camera_calibration_results_path + camera_extrinsic_filename[i], 'w') as file:
		data = yaml.dump(camera_extrinsics, file, default_flow_style=False)
		file.close()
	object_points = []
	image_points = []
	image_points_= []
	pbar.update(1)
	sleep(0.5)

pbar.close()
