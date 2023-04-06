#!/usr/bin/env python

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


qs = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
    history=QoSHistoryPolicy.KEEP_ALL
)

class Collect_Waypoints(Node):
    def __init__(self):
        super().__init__("Collect_Waypoints")
        #setting flag for pathpoint array generation after callback
        self.flag = 0
        #getting param from controllaunch.launch
        self.declare_parameter('WAYPOINTS_FILE', '/odom_waypoints.dat')
        path = os.path.dirname(os.path.abspath(__file__))
        WaypointFile = self.get_parameter("WAYPOINTS_FILE").get_parameter_value().string_value
        self.filename = path + WaypointFile
        self.pathArray = []
        #start point
        self.point = [0,0]
        self.subscription1 = self.create_subscription(Odometry,"/vehicle/odom1",self.quat_callback, qs) # SEE ROS1 WAYPOINT GITHUB FOR OTHER SUBSCRIBERS
        rate = self.create_rate(50)
        while rclpy.ok():
            if self.flag == 1:
                #x,y
                self.pathArray.append([self.point[0], self.point[1]])
            with open(self.filename, 'w') as file:
                for row in self.pathArray:
                    #writing to DAT File (location in lib folder of pkg)
                    file.write(','.join([str(x) for x in row]))
                    file.write('\n')
            rclpy.spin_once(self) #spinning in while loop due to issues with rclpy.spin() outside of node (tried threading,generic rclpy.spin())
        rate.sleep()
    
    def quat_callback(self,data,*args):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        #temp=utm.from_latlon(lat_,long_)	
        self.point[0] = x
        self.point[1] = y
        print (self.point[0],self.point[1], "\n")
        self.flag = 1

			
def main(args=None):
    rclpy.init(args=args)
    cw= Collect_Waypoints()
    

if __name__ == '__main__':
    main()

