#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from vehicle_control_mkz.msg import A9
#from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler as QOE
from tf_transformations import euler_from_quaternion as EFQ
import utm
import numpy as np
import sys
import pdb
from rclpy.node import Node 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
#from custom_msgs.msg import positionEstimate SEE DOCUMENTATION ON ROS2 FOR REAL
#from custom_msgs.msg import orientationEstimate SEE DOCUMENTATION ON ROS2 FOR REAL
import threading 

# The sim flag is used to determine which set of subscribers and callbacks to use
# This could be done with launch file where you load a parameter and then use that
# But this hopefully give you an idea of how to use the parameter when you get to that point

class SensorFusionNode(Node):
    def __init__(self, sim=True, rate=50):
        # Load the parameters in case you want to use them elsewhere
        self.sim = sim
        self.rate = rate
        self.or_flag = 0
        self.pos_flag = 0
        #Starting sensor
        super().__init__('Sensor_Fusion_Node')
        
        # Define QoS
        qs= QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        #FOR SIM ONLY 
        if self.sim:
            self.subscription1 = self.create_subscription(Odometry,"/vehicle/ground_truth_odom", self.sim_orientation_cb, 1)
            self.subscription2 = self.create_subscription(NavSatFix,"/vehicle/gps/fix", self.sim_position_cb, 1)
        
        # For Real MKZ
        if not self.sim:
            # Otherwise use another set of subscibers and callbacks
            pass
        
        #Publish ODOM TOPICS 
        self.publisher1 = self.create_publisher(Odometry,'/vehicle/odom1',1)
        self.publisher2 = self.create_publisher(A9,'/vehicle/odom2',1)
        
        # Define global output message variables
        self.odomQuat = Odometry()
        self.odomEuler = A9()
        
        # [x, y, yaw] just for display purposes
        self.veh_pose = [0.0, 0.0, 0.0]
        
        # Create timer to publish
        self.timer = self.create_timer(1/self.rate, self.publish)
        
        
    def publish(self):

        #### Publish 	
        if self.pos_flag == 1: 
            self.publisher1.publish(self.odomQuat)
        if self.or_flag == 1:
            self.publisher2.publish(self.odomEuler)
        
        #### Print
        self.get_logger().info("Publishing: {} (x,y,yaw)".format(self.veh_pose))
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
        



    def sim_orientation_cb(self, msg: Odometry):
        # Fill in the quaternion message
        self.or_flag =1
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
    
    # This is kennys prior method for converting the quaternion to euler in my format
    # def sim_orientation_cb(self, msg: Imu):
    #     # The IMU message already has a quaternion and the output of this node is also a quaternion
    #     # Instead you could just do this:
    #     # self.odomQuat.pose.pose.orientation = msg.orientation
    #     # And if you need to know yaw for some reason you can still get it from the quaternion
        
    #     # Convert from NED to ENU
    #     yaw = np.mod(2*np.pi + np.pi/2 - np.radians(msg.orientation.w),2*np.pi)
        
    #     # Convert to quaternion
    #     q=QOE(0.0, 0.0, yaw)
        
    #     # Place into message
    #     self.odomQuat.header = msg.header
    #     self.odomQuat.pose.pose.orientation.x = q[0]
    #     self.odomQuat.pose.pose.orientation.y = q[1]
    #     self.odomQuat.pose.pose.orientation.z = q[2]
    #     self.odomQuat.pose.pose.orientation.w = q[3]
        
    #     self.odomEuler.roll = 0.0
    #     self.odomEuler.pitch = 0.0
    #     self.odomEuler.yaw = yaw

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
