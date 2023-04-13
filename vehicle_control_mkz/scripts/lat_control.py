#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from steering_methods import SteeringMethods
from ros_callback import RosCallbackDefine
import yaml
import pdb
import os
import time 

#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 
#### CHANGE IN package.xml file

#### USAGE
#    SIMPLY IMPORT OR CREATE AN INSTACE OF CLASS LongController
#    or run it 

class LatController(Node):
	def __init__(self):
		super().__init__('LatController')
		path = os.path.dirname(os.path.abspath(__file__))

		# GETTING PARAM FOR WAYPOINT FILE NAME 
		self.declare_parameter('WAYPOINTS_FILE', '/odom_waypoints.dat')
		path = os.path.dirname(os.path.abspath(__file__))
		WaypointFile = self.get_parameter("WAYPOINTS_FILE").get_parameter_value().string_value
		print(WaypointFile)

		wpfile = path + WaypointFile 
		vehicle = "/config/MKZ.yaml" #YAML FILE NAME 
		#### READ YAML FILE	
		with open(path + vehicle,'r') as stream:
			data_loaded = yaml.safe_load(stream)
		#set params from YAML
		lookAhead = data_loaded['lookAhead']
		wheelBase = data_loaded['wheelBase']
		steeringRatio = data_loaded['steeringRatio']
		steerLim_upper = data_loaded['steerLim']['upper']
		steerLim_lower = data_loaded['steerLim']['lower']
		#### PARAMETERS FOR ADAPTIVE VELOCITY AND LOOKAHEAD	
		lMin = data_loaded['Adaptive']['lMin']
		lMax = data_loaded['Adaptive']['lMax']
		vMin = data_loaded['Adaptive']['vMin']
		vMax = data_loaded['Adaptive']['vMax']
		gamma = data_loaded['Adaptive']['gamma']
		ayLim = data_loaded['Adaptive']['ayLim']
		#### CREATE SteeringFeedForward Object. Pass in the waypoints
		steeringFF = SteeringMethods(wpfile,lookAhead,wheelBase,steeringRatio)	#Using default mkz values, otherwise specify them: lookAhead = 10.0, wheelBase = 2.85, steeringRatio = 14.8
		topic_helper = RosCallbackDefine(vehicle)		
		#### CREATE LOOP TO SPIN CALLBACK NODE #######
		#rate=self.create_rate(5.0) issues with rate.sleep, using time.sleep
		while rclpy.ok():
			# methodPurePursuit function 
			# will use default values, otherwise specify them: lookAhead=self.LA,WheelBase=self.WB,SteeringRatio=self.SR
			# Returns steercmd, curv, eastingBearing,relativeBearing, self.targetPoint
			rclpy.spin_once(topic_helper)
			states = topic_helper.return_states()
			if len(states) > 1:
				linearX = states[0]
				poseX = states[1]
				poseY = states[2]
				yaw = states[3]
				steercmd, curv, absoluteBearing, relativeBearing, targetPoint = steeringFF.methodPurePursuit(poseX,poseY,yaw)		# Calculate steering 
				steeringFF.methodAdaptiveLookAhead(lMin, lMax, gamma, poseX, poseY, yaw, linearX, curv)
                # set variable speed
				vCmd = steeringFF.methodAdaptiveVelocity(vMin, vMax, ayLim, curv)
                # limit the steering angle
				steercmd_f = self.sat_limit(steercmd,steerLim_lower,steerLim_upper)
				# Publish the steering cmd
				topic_helper.publish_vehicle_lat(steercmd)			
                # Publish the desired velocity			
                #topic_helper.setDesiredVelocity(vCmd)
				print("\n SteerCmd: {} \n Curv: {}".format(steercmd, curv))
			time.sleep(1/50)
	def sat_limit(self,val,low,upper):
		#steering angle control params
		if val >= upper:
			return upper
		elif val <= low:
			return low
		else:
			return val

    
if __name__=="__main__":
    rclpy.init()
    latcontroller= LatController()


    


