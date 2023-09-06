#!/usr/bin/env python3

# This script collects waypoints and saves them to textfiles
# Specifically, it collects gps coordinates to one file
#Code Translated from Kennjaro to ROS2, See https://github.com/Kchour/MKZ_controller/blob/controllers/vehicle_controllers/nodes/collect_waypoints for reference

import rclpy
import math
import numpy as np
import utm
from nav_msgs.msg import Odometry
from vehicle_control_mkz.msg import A9
import pdb
import matplotlib.pyplot as plt
import os
from rclpy.node import Node 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


qs= QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE, 
                durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
                history=QoSHistoryPolicy.KEEP_LAST, depth=1)

class Collect_Waypoints(Node):
    def __init__(self,rate=50):
        self.rate = rate
        super().__init__("Collect_Waypoints")
        #setting flag for pathpoint array generation after callback
        self.flag = 0
        #getting param from controllaunch.launch
        self.declare_parameter('WAYPOINTS_FILE', '/follow_waypoints.dat')
        path = os.path.dirname(os.path.abspath(__file__))
        WaypointFile = self.get_parameter("WAYPOINTS_FILE").get_parameter_value().string_value
        self.filename = path + WaypointFile
        self.pathArray = []
        #start point
        self.i = 0
        self.point = [0,0]
        self.subscription1 = self.create_subscription(Odometry,"/vehicle/odom1",self.quat_callback, qs) # SEE ROS1 WAYPOINT GITHUB FOR OTHER SUBSCRIBERS
        self.timer = self.create_timer(1/self.rate,self.service_callback)

    
    
    def quat_callback(self,data,*args):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        #temp=utm.from_latlon(lat_,long_)	
        self.point[0] = x
        self.point[1] = y
        self.pathArray.append([self.point[0], self.point[1]])
        self.i +=1
        if self.i > 10:
            self.Waypoint_record()
        else:
            pass

    def service_callback(self):
        self.get_logger().info('Waypoint_generation recieved,recording...')
    
    def Waypoint_record(self):
        with open(self.filename, 'w') as file:
            for row in self.pathArray:
                #writing to DAT File (location in lib folder of pkg)
                file.write(','.join([str(x) for x in row]))
                file.write('\n')
        self.i = 0
    

			
def main(args=None):
    rclpy.init(args=args)
    cw= Collect_Waypoints()
    rclpy.spin(cw)
    

if __name__ == '__main__':
    main()
