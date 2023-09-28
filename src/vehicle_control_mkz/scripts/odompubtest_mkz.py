#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from vehicle_control_mkz.msg import A9
#from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler as QOE
from tf_transformations import euler_from_quaternion as EFQ
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import utm
import numpy as np
import sys
import pdb
from rclpy.node import Node 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy



class SensorFusionNode(Node):
    def __init__(self, rate=50):
        super().__init__('Sensor_Fusion_Node')
        # Load the parameters in case you want to use them elsewhere
        self.declare_parameter('SIM', 'SIM')
        self.sim = self.get_parameter('SIM').get_parameter_value().string_value
        self.rate = rate
        self.or_flag = 0
        self.pos_flag = 0
        #Starting sensor
        super().__init__('Sensor_Fusion_Node')
        

        #Uncomment this if you want to use the QOS profile for sensors: 
        '''
        qs= QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        '''

        #FOR SIM ONLY: Sensor topics 
        if self.sim == 'SIM':
            self.subscription1 = self.create_subscription(Odometry,"/vehicle/ground_truth_odom", self.sim_orientation_cb, 1)
            self.subscription2 = self.create_subscription(NavSatFix,"/vehicle/gps/fix", self.sim_position_cb, 1)

        
        # For Real MKZ: Sensor Topics 
        if self.sim == 'REAL':
            # Otherwise use another set of subscibers and callbacks
            pass
        

        #Publish ODOM TOPICS, 1 = Odometry 
        self.publisher_OdometryTopic = self.create_publisher(Odometry,'/vehicle/odom1',1)
        self.publisher_msgScript = self.create_publisher(A9,'/vehicle/odom2',1)
        self.RVIZ_plugin_travelledpath = self.create_publisher(Path,'/vehicle/rviz_path',1)
        
        # Define global output message variables
        self.odomQuat = Odometry()
        self.odomEuler = A9()
        self.Path = Path()
   
        # [x, y, yaw] just for display purposes
        self.veh_pose = [0.0, 0.0, 0.0]
        
        # Create timer to publish
        self.timer = self.create_timer(1/self.rate, self.publish)
        
        
    def publish(self):

        #### Publish 	
        if (self.pos_flag == 1) and (self.or_flag ==1): 
            self.publisher_OdometryTopic.publish(self.odomQuat)
            self.publisher_msgScript.publish(self.odomEuler)
            self.RVIZ_plugin_travelledpath.publish(self.Path)
        
            #### Print
            #self.get_logger().info("Publishing: {} (x,y,yaw)".format(self.veh_pose))
            # self.get_logger().info("Publishing: {}".format(self.odomQuat))
            # self.get_logger().info("Publishing: {}".format(self.odomEuler))


    def sim_position_cb(self, msg: NavSatFix):
        # Convert from NED to ENU
        self. pos_flag = 1
        utm_pose = utm.from_latlon(msg.latitude, msg.longitude)
        
        # Fill in class variables
        self.odomEuler.x = utm_pose[0]
        self.odomEuler.y = utm_pose[1]
        self.odomEuler.z = msg.altitude
        
        # Fill in display print display variable
        self.veh_pose[0] = utm_pose[0]
        self.veh_pose[1] = utm_pose[1]

        self.odomQuat.pose.pose.position.x = utm_pose[0]	
        self.odomQuat.pose.pose.position.y = utm_pose[1]
        self.odomQuat.pose.pose.position.z = msg.altitude

        #RVIZ Path plugin 
        self.Path_pose = PoseStamped()
        self.Path.header.frame_id = "world"
        self.Path_pose.pose.position.x = utm_pose[0]
        self.Path_pose.pose.position.y = utm_pose[1]
        self.header = Header()
        self.Path_pose.header.frame_id = "world"
        self.Path.poses.append(self.Path_pose)

        if (msg.latitude == 0) or (msg.longitude == 0): #avoiding sensor timeout- preventing lat from publishing if no feedback is recieved 
        	self.pos_flag = 0
        



    def sim_orientation_cb(self, msg: Odometry):
        # Fill in the quaternion message
        self.or_flag =1 # Set flag to prevent publish without sensor feedback
        self.odomQuat.header = msg.header
        self.odomQuat.pose.pose.orientation = msg.pose.pose.orientation

        
        
        # Get the euler angles from the quaternion
        euler = EFQ([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        
        # Fill them into the message
        self.odomEuler.header = msg.header
        self.odomEuler.roll = euler[0]
        self.odomEuler.pitch = euler[1]
        self.odomEuler.yaw = euler[2]
     
        # Fill in display print display variable
        self.veh_pose[2] = euler[2]

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    sf = SensorFusionNode()
    
    # Spin for callbacks
    rclpy.spin(sf)
    
    # Destroy the node after Ctrl+C
    sf.destroy_node()
    rclpy.shutdown()
    sf.destroy_node()
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()
