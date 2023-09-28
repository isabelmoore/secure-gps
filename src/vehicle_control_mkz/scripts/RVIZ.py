#!/usr/bin/env python3

from nav_msgs.msg import Path
import os
import numpy as np 
from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, PoseStamped
import rclpy
from rclpy.node import Node
from steering_methods import SteeringMethods

from vehicle_control_mkz.msg import A9
'''
class RVIZ_Plugin(Node):
    def __init__(self,rate=50):
        super().__init__('RVIZ')
        self.i = 0
        self.rate = rate
        self.timer = self.create_timer(1/self.rate, self.publish)
        # WAYPOINT PROCESSOR
        self.declare_parameter('WAYPOINTS_FILE', '/odom_waypoints.dat')
        self.path = os.path.dirname(os.path.abspath(__file__))
        self.WaypointFile = self.get_parameter("WAYPOINTS_FILE").get_parameter_value().string_value

        # JOIN WAYPOINT PATH TO OS 
        self.wpfile = self.path + self.WaypointFile 
        self.path_array = []

        with open(self.wpfile, "r") as f:
            for line in f:
                self.path_array.append(line.strip())

        self.path_array = np.array([list(map(float, x.split(','))) for x in self.path_array])
        self.path_array = np.array([[float(y) for y in x] for x in self.path_array])

        # Def Publisher 

        # Create publisher for RVIZ disp 
        self.RVIZ_Path = Path()
        self.PoseStampedMsg = PoseStamped()
        ##

        self.RVIZ_Path.poses.append()

        self.publisher_RVIZ1 = self.create_publisher(Path, '/vehicle/desired_rviz_path', 1)

        
    def waypoint_store_RVIZ(self):
        self.i += 1
        # This is for showing the full path on RVIZ 
        self.SM = SteeringMethods(self.wpfile)
        self.path_array = self.SM.RVIZ_plugin()
        self.poses = PoseArray()


        # TODO
        if self.i > (len(self.path_array) - 1):
            self.i = 0
            self.poses.pose.position.x = float(self.path_array[self.i][0])
            self.poses.pose.position.y = float(self.path_array[self.i][1])
        else:
            self.poses.pose.position.x = float(self.path_array[self.i][0])
            self.poses.pose.position.y = float(self.path_array[self.i][1])

            self.Header = Header()
            self.RVIZ_Path.header.frame_id = "world"
            self.RVIZ_Path.poses.append(self.poses)

    def publish(self):
        self.waypoint_store_RVIZ()

        # Def display for RVIZ
        self.publisher_RVIZ1.publish(self.RVIZ_Path)

def main(args=None):
    rclpy.init(args=args)
    rviz = RVIZ_Plugin()
    rclpy.spin(rviz)
    rviz.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''