#!/usr/bin/env python3

#ROS2 TOPICS 
import rclpy
from rclpy.node import Node

#Steering methods import for vehicle POS 
from steering_methods import SteeringMethods
from dbw_ford_msgs.msg import SteeringCmd

#Msg topic for Waypoints 
from vehicle_control_mkz.msg import A9

#topics for RVIZ plugins 
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

#Path working directory 
import yaml
import os

#QOS 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
# FOR RVIZ Display

from visualization_msgs.msg import Marker
from std_msgs.msg import Header



class LatController(Node):
	def __init__(self,sim=True,rate=50):
		'''
		Uncomment this if you want to use the QOS profile for sensor topics 
		qs= QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
		 durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
		 history=QoSHistoryPolicy.KEEP_LAST, depth=1)
		'''
	
		self.sim = sim
		self.rate = rate
		### init node##
		super().__init__('lat_controller_node')
		#DEF SELF FLAG 
		#Flags for CB
		self.cb_flag = [0,0]
		# GETTING PARAM FOR WAYPOINT FILE NAME 
		self.declare_parameter('WAYPOINTS_FILE', '/odom_waypoints.dat')
		self.path = os.path.dirname(os.path.abspath(__file__))
		self.WaypointFile = self.get_parameter("WAYPOINTS_FILE").get_parameter_value().string_value
		#JOIN WAYPOINT PATH TO OS 
		self.wpfile = self.path + self.WaypointFile 

		#YAML
		self.vehicle = "/config/MKZ.yaml" #YAML FILE NAME 
		#### READ YAML FILE	
		with open(self.path + self.vehicle,'r') as stream:
			data_loaded = yaml.safe_load(stream)
		#set params from YAML
		lookAhead = data_loaded['lookAhead']
		wheelBase = data_loaded['wheelBase']
		steeringRatio = data_loaded['steeringRatio']
		self.steerLim_upper = data_loaded['steerLim']['upper']
		self.steerLim_lower = data_loaded['steerLim']['lower']
		#### PARAMETERS FOR ADAPTIVE VELOCITY AND LOOKAHEAD	
		self.lMin = data_loaded['Adaptive']['lMin']
		self.lMax = data_loaded['Adaptive']['lMax']
		self.vMin = data_loaded['Adaptive']['vMin']
		self.vMax = data_loaded['Adaptive']['vMax']
		self.gamma = data_loaded['Adaptive']['gamma']
		self.ayLim = data_loaded['Adaptive']['ayLim']

		#Global var for RVIZ path
		self.RVIZ_Path = Path()
		self.PoseStampedMsg = PoseStamped()

		#Publish msg 
		self.publisher_RVIZ1 = self.create_publisher(Path, '/vehicle/desired_rviz_path', 1)

		#args
		self.steeringMsg = SteeringCmd()
		self.steeringMsg.enable = True
		##

		#Get steering angle
		self.steeringFF = SteeringMethods(self.wpfile,lookAhead,wheelBase,steeringRatio)
		self.get_logger().info("Before")
		self.path_array = self.steeringFF.RVIZ_plugin()
		self.get_logger().info("After")

		#Get desired RVIZ path 
		self.Waypoint_plotter_rviz()

		#subscribers
		self.subOdom = self.create_subscription(A9,'/vehicle/odom2',self.__odom_cb,1)
		self.subspeed =self.create_subscription(TwistStamped,"/vehicle/twist",self.lat_speed_cb,1)

		##Flags to prevent publishing until sensor feedback is met###
		states = [0]
		self.flag= 0
		#def rate for publish
		self.timer = self.create_timer(1/self.rate, self.publish)
	
	






	def return_states(self):
		if self.cb_flag == [1,1]:

			states = [self.linearX,self.pose_x,self.pose_y,self.yaw]
			self.flag = 1 

		else:
			states = [0]

			
		###CALLING APPROPRIATE FUNCTIONS IF LENGTH IS VALID
		if len(states) >= 4:

			self.pubSteer = self.create_publisher(SteeringCmd,'/vehicle/steering_cmd',1)
			self.steering(states,self.steeringFF)
			self.flag = 1
		else:
			pass


	def __odom_cb(self,msg):

		self.pose_x = msg.x#msg.pose.pose.position.x
		self.pose_y = msg.y#msg.pose.pose.position.y
		#quat = msg.pose.pose.orientation
		#euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
		self.roll = msg.roll # euler[0]
		self.pitch = msg.pitch # euler[1]
		self.yaw = msg.yaw #euler[2]
		self.cb_flag[0] = 1
	
	def lat_speed_cb(self,msg):
		self.linearX = msg.twist.linear.x
		self.cb_flag[1] = 1 
	

	def publish(self):
			self.return_states()
			###
			if self.flag == 1:
				self.steeringMsg.steering_wheel_angle_cmd = self.steercmd_f
				self.pubSteer.publish(self.steeringMsg)	
				self.publisher_RVIZ1.publish(self.RVIZ_Path)
				#self.get_logger().info("Publishing Lateral Controller")


	def steering(self,states,steeringFF):
		#Steering Angle
		self.steercmd,self.curv,self.absoluteBearing,self.relativeBearing,self.targetPoint = steeringFF.methodPurePursuit(states[1],states[2],states[3])
		#get waypoints from Steering methods script
		#self.vCmd = steeringFF.methodAdaptiveVelocity(vMin, vMax, ayLim, curv)
		steeringFF.methodAdaptiveLookAhead(self.lMin, self.lMax, self.gamma, states[1], states[2], states[3], states[0], self.curv)
		#limit steer angle 
		self.steercmd_f = self.sat_limit(self.steercmd,self.steerLim_lower,self.steerLim_upper)
		#print("\n SteerCmd: {} \n Curv: {}".format(self.steercmd_f, self.curv)) uncomment for debugging 


	def sat_limit(self,val,low,upper):
		#steering angle control params
		if val >= upper:
			return upper
		elif val <= low:
			return low
		else:
			return val
		
	def Waypoint_plotter_rviz(self):
		for i in range(len(self.path_array) - 1):
			self.header = Header()
			self.RVIZ_Path.header.frame_id = "world"
			#self.RVIZ_Path.poses = []
			self.PoseStampedMsg.pose.position.x = float(self.path_array[i][0])
			self.PoseStampedMsg.pose.position.y = float(self.path_array[i][1])
			self.PoseStampedMsg.header.frame_id = "world"
			self.RVIZ_Path.poses.append(self.PoseStampedMsg)
			i += 10
		self.get_logger().info("Complete")

	

def main(args=None):
	rclpy.init(args=args)
	latc= LatController()
	rclpy.spin(latc)
		
	# Create the node
	# Spin for callback
		
	# Destroy the node after Ctrl+C
	latc.destroy_node()
	rclpy.shutdown()

			



if __name__=="__main__":
    main()



    


