#!/usr/bin/env python3

# This script collects waypoints and saves them to a text file
# Specifically, it collects gps coordinates to one file
#Code Translated from Kennjaro to ROS2, See https://github.com/Kchour/MKZ_controller/blob/controllers/vehicle_controllers/nodes/collect_waypoints for reference

import rclpy
from rclpy.node import Node 
from nav_msgs.msg import Odometry


class VehiclePathRecorder(Node):
    def __init__(self):
        # Initialize the node
        super().__init__("vehicle_path_recorder")
        
        # Get the path for the waypoints file
        self.declare_parameter('waypoints_file_path', 'waypoints.dat')
        self.waypoints_file_path = self.get_parameter("waypoints_file_path").get_parameter_value().string_value
        self.get_logger().info("Waypoint File: " + self.waypoints_file_path)
        
        # Get the write rate from the launch file
        self.declare_parameter('write_rate', 2.0)
        write_rate = self.get_parameter('write_rate').get_parameter_value().double_value
        self.get_logger().info("Write Rate: " + str(write_rate) + " hz")
        
        # Path array for storing the points
        self.pathArray = []
        
        # Subscribe to the odometry topic for the vehicle location
        self.odom_sub = self.create_subscription(Odometry,"/vehicle/odom1",self.odom_callback, 1)
        
        # create a timer to trigger the write
        self.create_timer(1/write_rate, self.write_points_callback)
        
    # Update the path array with the latest point on callback of the odometry
    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pathArray.append([x, y])

    # When timer is called, write the points to file
    def write_points_callback(self):
        # Open file for writing, iterate through points and write to file
        with open(self.waypoints_file_path, 'w') as file:
            for row in self.pathArray:
                file.write(','.join([str(x) for x in row]))
                file.write('\n')
    
def main(args=None):
    rclpy.init(args=args)
    vr = VehiclePathRecorder()
    rclpy.spin(vr)
    

if __name__ == '__main__':
    main()
