#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
import tf
import pdb
import numpy as np
from threading import Thread, Lock

###MKZ MSGS
from dbw_ford_msgs.msg import SteeringCmd
from dbw_ford_msgs.msg import ThrottleCmd
from dbw_ford_msgs.msg import BrakeCmd
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

from vehicle_control_mkz.msg import A9


wpmutex = Lock()

global timeBefore
timeBefore = 0.0

qs= QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
    history=QoSHistoryPolicy.KEEP_ALL
)

class RosCallbackDefine(Node):
	def __init__(self,*args):
		#### LONGITUDINAL TOPICS #### 
		# Init Node cmds
		super().__init__('ROS_VEHICLE_CALLBACK')
		#callback flags
		self.flag = [0,0]
		#DEF PUBLISHERS/SUBSCRIBERS FOR LAT/LONG CONTROLLERS
		self.__init_mkz()
		#SUBSCRIBE TO ODOM MSG

	#veh position from sim
	def return_states(self):
		if sum(self.flag) == len(self.flag):
			return [self.linearX, self.pose_x, self.pose_y, self.yaw]
		else:
			return [0]
	
	def return_waypoints(self):
		#NOT SURE WHERE THIS IS CALLED, SO FAR UNUSED 
		global timeBefore
		wpmutex.aquire()
		self.return_waypoints_temp = list(self.waypoints)
		wpmutex.release()
		if len(self.return_waypoints_temp) > 0:
			return np.array(self.return_waypoints_temp)
		else:

			return [0]
		
	def publish_vehicle_lat(self,steering):
		#LATERAL CONTROLLER PUBLISHES TO STEERING ANGLE
		self.steeringMsg.steering_wheel_angle_cmd = steering
		self.pubSteer.publish(self.steeringMsg)	

	def publish_vehicle_long(self,throttle,brake):
		#CONTROLS BRAKE AND THROTTLE
		self.throttleMsg.pedal_cmd = throttle
		self.brakeMsg.pedal_cmd = brake
		#### PUBLISH MESSAGES		
		self.pubThrottle.publish(self.throttleMsg)
		self.pubBrake.publish(self.brakeMsg)




	
	def __speed_cb(self,msg,*args):
		#### MKZ CASE ####
		self.linearX = msg.twist.linear.x
		self.flag[0] = 1

	def __odom_cb(self,msg,*args):

		self.pose_x = msg.x#msg.pose.pose.position.x
		self.pose_y = msg.y#msg.pose.pose.position.y
		#quat = msg.pose.pose.orientation
		#euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.roll = msg.roll # euler[0]
		self.pitch = msg.pitch # euler[1]
		self.yaw = msg.yaw #euler[2]
		#self.linearX = msg.twist.twist.linear.x
		#self.angularZ = msg.twist.twist.angular.z	
		self.flag[1] = 1
	
	def __odom_cb_filtered(self,msg):
		self.pose_x = msg.pose.pose.position.x
		self.pose_y = msg.pose.pose.position.y
		quat = msg.pose.pose.orientation
		euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.roll = euler[0]
		self.pitch = euler[1]
		self.yaw = euler[2]
        # odometry not using it atm
        #self.linearX = msg.twist.twist.linear.x
		self.angularZ = msg.twist.twist.angular.z
		self.flag[1] = 1
	#NOT SURE WHERE THIS IS CALLED, SO FAR UNUSED 
	def __waypoints_cb(self,msg):
		wpmutex.acquire()
		self.waypoints = []
		for i in range(len(msg.poses)):
			poses_ind = msg.poses[i].pose.position
			self.waypoints.append([poses_ind.x, poses_ind.y])
		wpmutex.release()
	#NOT SURE WHERE THIS IS CALLED, SO FAR UNUSED 
	def setDesiredVelocity(self, vCmd):
		self.setVelMsg.data = vCmd
		self.setVelPub.publish(self.setVelMsg)
		
	#### MKZ INITIALIZATION OF PUB/SUB ####
	def __init_mkz(self):

		self.longflag = 1
		#### CREATE PUBLISHING MESSAGES ###
		self.throttleMsg = ThrottleCmd()
		self.brakeMsg = BrakeCmd()
		self.steeringMsg = SteeringCmd()
		self.setVelMsg = Float64()	
		
		self.throttleMsg.enable = True
		self.throttleMsg.pedal_cmd_type = 2
		self.brakeMsg.enable = True
		self.brakeMsg.pedal_cmd_type = 2
		self.steeringMsg.enable = True

		### LONG TOPICS###
		self.subspeed = self.create_subscription(TwistStamped,"/vehicle/twist",self.__speed_cb, qs)
		self.pubThrottle = self.create_publisher(ThrottleCmd,'/vehicle/throttle_cmd',1)
		self.pubBrake = self.create_publisher(BrakeCmd,'/vehicle/brake_cmd',1)
		# COMMAND THROTTLE TOPIC/COMMAND BRAKE TOPICS PUBLISHED IN THIS SCRIPT

		#### LATERAL TOPICS #### 
		self.pubSteer = self.create_publisher(SteeringCmd,'/vehicle/steering_cmd',1)
		self.subOdom = self.create_subscription(A9,'/vehicle/odom2',self.__odom_cb,qs)
		### Adjusting Desired Velocity (didn't see where this is called)
		self.setVelPub = self.create_publisher(Float64,"/long_controller/cmd_vel",1)
