#!/usr/bin/env python
import rclpy
from rclpy.node import Node
import os
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import csv
import math
from darknet_ros_msgs.msg import BoundingBoxes
from swiftnav_ros2_driver.msg import Baseline
 
class ControlPointRecocrder(Node):
	def __init__(self):
		super().__init__('Control_Point_collector')

		self.control_point_path = os.getcwd() +r'/src/infraware_ros/control_points/'
		self.file1 = open(self.control_point_path + 'controlPoint1.txt',"w")
		self.file2 = open(self.control_point_path + 'controlPoint2.txt',"w")


		self.x_pixel=[0.000,0.000]
		self.x_pixel2=[0.000,0.000]
		self.x_pixel3=[0.000,0.000]
		self.x_pixel4=[0.000,0.000]
		self.X_world=[0.000,0.000,0.000]

		self.xmin1_=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.xmin2_=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.xmin3_=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.xmin4_=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.ymin1_=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.ymin2_=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.ymin3_=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.ymin4_=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]


		self.create_subscription(BoundingBoxes, '/camera1/bounding_boxes',  self.darknet1_callback,10)
		self.create_subscription(BoundingBoxes, '/camera2/bounding_boxes',  self.darknet2_callback,10)
		self.create_subscription(BoundingBoxes, '/camera3/bounding_boxes',  self.darknet3_callback,10)
		self.create_subscription(BoundingBoxes, '/camera4/bounding_boxes', self.darknet4_callback,10)

		self.create_subscription(Baseline, '/baseline', self.pixie_position_callback,10)



	def pixie_position_callback(self,data):
		self.X_world=[data.baseline_n_m,data.baseline_e_m,data.baseline_d_m]
                #if the data are on one line, the PNP solver can't handle different oreintaion in image fram and world frame. Thus, the x and y should change to match the image frame rientation.



	def darknet1_callback(self,data):
		distance =0 
		for i in range(len(data.bounding_boxes)):
			distance = (data.bounding_boxes[i].xmin - self.xmin1_[i])**2 + (data.bounding_boxes[i].ymin - self.ymin1_[i])**2
			self.xmin1_[i]= data.bounding_boxes[i].xmin
			self.ymin1_[i]= data.bounding_boxes[i].ymin
			if (data.bounding_boxes[i].class_id == "car" and distance>50):
				self.x_pixel = [0.5*(data.bounding_boxes[i].xmin+data.bounding_boxes[i].xmax),
		                data.bounding_boxes[i].ymax]
				self.write_to_file(self.X_world,self.x_pixel,self.file1)

	def darknet2_callback(self,data):
		distance =0 
		for i in range(len(data.bounding_boxes)):
			distance = (data.bounding_boxes[i].xmin - self.xmin2_[i])**2 + (data.bounding_boxes[i].ymin - self.ymin2_[i])**2
			self.xmin2_[i]= data.bounding_boxes[i].xmin
			self.ymin2_[i]= data.bounding_boxes[i].ymin
			if (data.bounding_boxes[i].class_id == "car" and distance>50):
				self.x_pixel2 = [0.5*(data.bounding_boxes[i].xmin+data.bounding_boxes[i].xmax),
		                data.bounding_boxes[i].ymax]
				self.write_to_file(self.X_world,self.x_pixel2,self.file2)	
        '''
	def darknet3_callback(self,data):
		distance =0

		for i in range(len(data.bounding_boxes)):
			distance = (data.bounding_boxes[i].xmin - self.xmin3_[i])**2 + (data.bounding_boxes[i].ymin - self.ymin3_[i])**2
			self.xmin3_[i]= data.bounding_boxes[i].xmin
			self.ymin3_[i]= data.bounding_boxes[i].ymin           
			if (data.bounding_boxes[i].class_id == "car" and distance>50):
				self.x_pixel3 = [0.5*(data.bounding_boxes[0].xmin+data.bounding_boxes[0].xmax),
		                0.5*(data.bounding_boxes[0].ymin+data.bounding_boxes[0].ymax)]
				self.write_to_file(self.X_world,self.x_pixel3,self.file3)
	def darknet4_callback(self,data):
		distance =0 
		for i in range(len(data.bounding_boxes)):
			distance = (data.bounding_boxes[i].xmin - self.xmin4_[i])**2 + (data.bounding_boxes[i].ymin - self.ymin4_[i])**2
			self.xmin4_[i]= data.bounding_boxes[i].xmin
			self.ymin4_[i]= data.bounding_boxes[i].ymin
			if (data.bounding_boxes[i].class_id == "car" and distance>50):
				self.x_pixel4 = [0.5*(data.bounding_boxes[0].xmin+data.bounding_boxes[0].xmax),
		                0.5*(data.bounding_boxes[0].ymin+data.bounding_boxes[0].ymax)]
				self.write_to_file(self.X_world,self.x_pixel4,self.file4)
        '''

	def write_to_file(self,X_world,x_pixel,file):

		file.write(str(X_world[0]) +","+ str(X_world[1]) +","+ str(X_world[2]) +","+ str(x_pixel[0]) +","+ str(x_pixel[1])+"\n")



def main():
	rclpy.init()
	#control_point_path = os.path.dirname(os.getcwd()) + r'/control_points/'

	node=ControlPointRecocrder()
	rclpy.spin(node)

if __name__ == "__main__":
        main()
