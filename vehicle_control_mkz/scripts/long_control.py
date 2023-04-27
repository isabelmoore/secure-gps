#!/usr/bin/env python
import rclpy
from rclpy.node import Node
#from dbw_mkz_msgs.msg import BrakeCmd
#from dbw_mkz_msgs.msg import ThrottleCmd
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from pid import PID
from ros_callback import RosCallbackDefine
from low_pass_filter import LinearFilter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
import pdb
import yaml
import os
import threading
import inspect
import time 
#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 
####
from geometry_msgs.msg import TwistStamped
from dbw_ford_msgs.msg import ThrottleCmd
from dbw_ford_msgs.msg import BrakeCmd
from vehicle_control_mkz.msg import A9
####
#### USAGE
#    SIMPLY IMPORT AND CREATE AN INSTANCE OF CLASS LongController
#    or run it 


class LongController(Node):
	def __init__(self, sim=True, rate=50):
		###set simulation/rate###
		self.sim = sim
		self.rate = rate
		self.state_flag = 0
		### init node##
		super().__init__('long_controller_node')
		#DEF SELF FLAG 
		self.flag = 0
		#Read YAML File from script 
		path = os.path.dirname(os.path.abspath(__file__))
		self.vehicle = "/config/MKZ.yaml" #yAML FILE NAME, THIS ONE IS SIMULATION FILE I BELIEVE
		
		qs= QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
		 durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
		 history=QoSHistoryPolicy.KEEP_ALL)
		
		#### READ YAML FILE	
		with open(path + self.vehicle,'r') as stream:
			data_loaded = yaml.safe_load(stream)
		#### large error throttle
		self.P1 = data_loaded['PID1']['P']
		self.I1 = data_loaded['PID1']['I']
		self.D1 = data_loaded['PID1']['D']
		#### Large Error Brake
		self.P2 = data_loaded['PID2']['P']
		self.I2 = data_loaded['PID2']['I']
		self.D2 = data_loaded['PID2']['D']
		#### Small Error Brake
		self.P3 = data_loaded['PID3']['P']
		self.I3 = data_loaded['PID3']['I']
		self.D3 = data_loaded['PID3']['D']
		self.errTol = data_loaded['Tol']
		###
		self.PLIST_throttle = [self.P1,self.I1,self.D1]
		self.PLIST_LEBrake = [self.P2,self.I2,self.D2]
		self.PLIST_SEBrake = [self.P3,self.I3,self.D3]

		
		### DECLARE PARAM FOR DESIRED SPEED
		self.declare_parameter('DESIRED_SPEED', 6.0)
		vx_input = float(self.get_parameter('DESIRED_SPEED').value)
		self.vx_desired = vx_input

		### Longitudinal publishers/sub from ROS callback
		self.subspeed = self.create_subscription(TwistStamped,"/vehicle/twist",self.__speed_cb, 1)
		#########################

		########
		#TIMER CREATE
		self.timer = self.create_timer(1/self.rate, self.publish)


		#global throttle/brake
		self.throttle_cmd = 0.0
		self.brake_cmd = 0.0
		self.brakeMsg = BrakeCmd()
		self.throttleMsg = ThrottleCmd()
		self.throttleMsg.enable = True
		self.throttleMsg.pedal_cmd_type = 2
		self.brakeMsg.enable = True
		self.brakeMsg.pedal_cmd_type = 2


		#PID
		self.PIDcontroller=PID()
		self.PIDcontroller.setLims(0.0,1000.)
		#self.velFilter = LinearFilter(0.05,[1,-0.95])
		self.throttleFilter = LinearFilter(0.05,[1,-0.95])
		self.brakeFilter = LinearFilter(0.05,[1,-0.95])
		###

		######START READING CONTROLLER NODES
		states = []
		###CALLING APPROPRIATE FUNCTIONS IF LENGTH IS VALID
		if len(states) != 0:
			print("PASSED_TRUE")
			self.publishTrue
			self.pubThrottle = self.create_publisher(ThrottleCmd,'/vehicle/throttle_cmd',1)
			self.pubBrake = self.create_publisher(BrakeCmd,'/vehicle/brake_cmd',1)
		else:
			pass

	def publish(self):
		print("CB")
		self.return_states()
		###
		if self.flag == 1:
			self.throttleMsg.pedal_cmd = self.throttle_cmd
			self.brakeMsg.pedal_cmd = self.brake_cmd
			self.pubThrottle.publish(self.throttleMsg)
			self.pubBrake.publish(self.brakeMsg)
			self.get_logger().info("Publishing Longitudinal Controller")
		else:
			pass 
	
	def return_states(self):
		if self.state_flag == 1:
			print("return_States")
			states = [self.linearX]
			self.pubThrottle = self.create_publisher(ThrottleCmd,'/vehicle/throttle_cmd',1)
			self.pubBrake = self.create_publisher(BrakeCmd,'/vehicle/brake_cmd',1)
			self.publishTrue(states,self.PIDcontroller,self.errTol,self.PLIST_throttle,self.PLIST_LEBrake,self.PLIST_SEBrake,self.throttleFilter,self.brakeFilter)
			self.flag = 1 

		else:
			states = [0]

		
	def __speed_cb(self,msg,*args):
		#### MKZ CASE ####
		self.linearX = msg.twist.linear.x
		print(self.linearX)
		self.state_flag = 1 

	
	def publishTrue(self,states,PIDcontroller,errTol,PLIST_throttle,PLIST_LEBrake,PLIST_SEBrake,throttleFilter,brakeFilter):
		#GETTING RID OF CALLBACK SCRIPT FOR SIMPLIFICATION- MKZ 
			vxError = self.vx_desired - states[0]
			PIDcontroller.update(vxError)
			if vxError >= -1.0:   
				print("Throttle")
				PIDcontroller.setGains(PLIST_throttle[0]*vxError,PLIST_throttle[1],PLIST_throttle[2]*vxError)
				u = PIDcontroller.computeControl()      # Compute Control input
				u = self.satValues(u,0.0,1.0)           # Bound control input

				self.throttle_cmd = throttleFilter.update_filter(u)
				self.brake_cmd = 0.0                # set brake to 0

			elif vxError < -1.0:
				if abs(vxError) >= errTol:
					print("Brake")
					PIDcontroller.setGains(PLIST_LEBrake[0]*vxError,PLIST_LEBrake[1],PLIST_LEBrake[2]*vxError)
					u = PIDcontroller.computeControl()      # Compute Control input
					u = self.satValues(u,0.0,1.0)           # Bound control input
					#throttle_cmd = u               # set throttle
					#throttle_cmd = self.lowpass(throttle_cmd,throttle_cmd_prev,0.96)
					#throttle_cmd_prev = throttle_cmd
					self.throttle_cmd = brakeFilter.update_filter(u)
					self.brake_cmd = 0.0                # set brake to 0
				else:
					PIDcontroller.setGains(PLIST_SEBrake[0],PLIST_SEBrake[1],PLIST_SEBrake[2])
					u = PIDcontroller.computeControl()           # Compute Control input
					u = self.satValues(abs(u),0.0,1.0)           # Bound control input
					#brake_cmd = u                  # set brake
					#brake_cmd = self.lowpass(brake_cmd,brake_cmd_prev,0.50)
					#brake_cmd_prev = brake_cmd
					self.brake_cmd = brakeFilter.update_filter(u)
					self.throttle_cmd = 0.0             # set throttle to 0

				#### PUBLISH THE MESSAGES in PUBLISH FUNCTION 
		

				####  MESSAGES FOR DEBUG
			print(vxError)
			print("\n speed cmd: {} \n speed: {} \n throttle: {} \n brake: {} \n totalError: {}".format(self.vx_desired,states[0],self.throttle_cmd,self.brake_cmd,PIDcontroller.errorTotalReturn()))

	def satValues(self,value,satLower, satUpper):

		if value >= satUpper:
			return satUpper
		elif value <= satLower:
			return satLower
		else:
			return value
	
	def lowpass(self,val,prev,alp):
		return alp*val + (1-alp)*prev

    
if __name__=="__main__":
    rclpy.init()
    lc = LongController()
    rclpy.spin(lc)