#!/usr/bin/env python
import rclpy
from rclpy.node import Node
#from dbw_mkz_msgs.msg import BrakeCmd
#from dbw_mkz_msgs.msg import ThrottleCmd
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped
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

#### USAGE
#    SIMPLY IMPORT AND CREATE AN INSTANCE OF CLASS LongController
#    or run it 

qs= QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
    history=QoSHistoryPolicy.KEEP_ALL
)

class FunctionOnce:
	def __init__(self):
		self.Once = False
	def set_flag(self,case):
		self.Once = case
	def return_flag(self):
		return self.Once

class LongController(Node):
	def __init__(self,*args):
		super().__init__('long_controller_node')
		#### INITIALIZE DESIRED VEHICLE SPEED
		self.declare_parameter('DESIRED_SPEED', 3.0)
		vx_input = float(self.get_parameter('DESIRED_SPEED').value)
		self.vx_desired = vx_input
		#### READ YAML FILE from script path
		path = os.path.dirname(os.path.abspath(__file__))
		self.vehicle = "/config/MKZ.yaml" #yAML FILE NAME, THIS ONE IS SIMULATION FILE I BELIEVE
		#### READ YAML FILE	
		with open(path + self.vehicle,'r') as stream:
			data_loaded = yaml.safe_load(stream)
		#### large error throttle
		P1 = data_loaded['PID1']['P']
		I1 = data_loaded['PID1']['I']
		D1 = data_loaded['PID1']['D']
		#### Large Error Brake
		P2 = data_loaded['PID2']['P']
		I2 = data_loaded['PID2']['I']
		D2 = data_loaded['PID2']['D']
		#### Small Error Brake
		P3 = data_loaded['PID3']['P']
		I3 = data_loaded['PID3']['I']
		D3 = data_loaded['PID3']['D']
		errTol = data_loaded['Tol']

		self.subCmd = self.create_subscription(Float64,'long_controller/cmd_vel',self.cmd_cb,qs)
		#PID
		PIDcontroller=PID()
		PIDcontroller.setLims(0.0,1000.)
		velFilter = LinearFilter(0.05,[1,-0.95])
		throttleFilter = LinearFilter(0.05,[1,-0.95])
		brakeFilter = LinearFilter(0.05,[1,-0.95])
		#### CREATE FLAG OBJECT
		smallErrorFlag = FunctionOnce()

		topic_helper = RosCallbackDefine("MKZ")

	
		#### CREATE TOPIC HELPER CLASS
		#### PUBLISH LOOP and Additional parameters

		#rate=self.create_rate(5.0) issues with rate.sleep, using time.sleep
		while rclpy.ok:
			rclpy.spin_once(topic_helper,*args)
  	    	#### RETURN STATES
			#### Gain scheduling for different regions of error
			
			states = topic_helper.return_states()

			if len(states) > 1:
					linearX = states[0]
					pose_x = states[1]
					pose_y = states[2]
					yaw = states[3]
					publishTrue = True
	
			else:
					publishTrue = False
			
			if publishTrue == True:

				#### Enter loop only when callbacks happen
				#linearX = self.lowpass(linearX,linearX_prev,0.50)
				#linearX_prev = linearX
				vxError = self.vx_desired - linearX 
	
				PIDcontroller.update(vxError)
				print(vxError)

				if vxError >= -1.0:   

					PIDcontroller.setGains(P1*vxError,I1,D1*vxError)
					u = PIDcontroller.computeControl()      # Compute Control input
					u = self.satValues(u,0.0,1.0)           # Bound control input
					#throttle_cmd = u               # set throttle
					#throttle_cmd = self.lowpass(throttle_cmd,throttle_cmd_prev,0.96)
					#throttle_cmd_prev = throttle_cmd
					throttle_cmd = throttleFilter.update_filter(u)
					brake_cmd = 0.0                # set brake to 0
				elif vxError < -1.0:

					smallErrorFlag.set_flag(False)
					if abs(vxError) >= errTol:
						PIDcontroller.setGains(P2,I2,I2)
					else:
						PIDcontroller.setGains(P3,I3,D3)
					u = PIDcontroller.computeControl()           # Compute Control input
					u = self.satValues(abs(u),0.0,1.0)           # Bound control input
					#brake_cmd = u                  # set brake
					#brake_cmd = self.lowpass(brake_cmd,brake_cmd_prev,0.50)
					#brake_cmd_prev = brake_cmd
					brake_cmd = brakeFilter.update_filter(u)
					throttle_cmd = 0.0             # set throttle to 0

				#### PUBLISH THE MESSAGES
				topic_helper.publish_vehicle_long(throttle_cmd,brake_cmd)
				
				#### PRINT MESSAGES FOR DEBUG
				#print("\n speed cmd: {} \n speed: {} \n throttle: {} \n brake: {} \n totalError: {}".format(self.vx_desired,linearX,throttle_cmd,brake_cmd,PIDcontroller.errorTotalReturn()))
			time.sleep(1/15)
	def cmd_cb(self,msg):
		#Not sure when or if cmd_cb is called back
		self.vx_desired = msg.data
    #-----------------------------

    #### MISC FUNCTIONS
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

    longcontroller = LongController()
    thread = threading.Thread(target=rclpy.spin, args=(LongController, ), daemon=True)
    thread.start()

