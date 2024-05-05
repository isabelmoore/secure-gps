#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import random
import math
from geographiclib.geodesic import Geodesic


class SpoofManipulate(Node):

    def __init__(self):
        super().__init__('spoof_node')
        
        # self.subscription = self.create_subscription(NavSatFix, '/vehicle/gps/fix', self.callback, 10)
        self.gps_subscription = self.create_subscription(NavSatFix,"/vectornav/gnss", self.gps_callback, 10)

        self.global_publisher = self.create_publisher(NavSatFix, '/manipulated_gps/global', 10)
        self.local_publisher = self.create_publisher(Odometry, '/manipulated_gps/local', 10)

        # Store the latest GPS data
        self.true_longitude = None
        self.true_latitude = None
        self.manipulated_longitude = None
        self.manipulated_latitude = None
        self.offset = 0

        # Timer for periodic logging
        self.log_timer = self.create_timer(1.0, self.log_gps_data)  # Adjust the interval as needed

    def random_bearing(self):
        # Generate two random bearings: one for latitude and one for longitude
        bearing_lat = random.uniform(-90, 90)
        bearing_long = random.uniform(-180, 180)
        return bearing_lat, bearing_long
    
    def gps_callback(self, msg):

        ''' Random Spherical Coordinates -- Following Dynamics '''

        self.true_longitude = msg.longitude
        self.true_latitude = msg.latitude
        print(self.true_latitude)

        # Generate random bearings for latitude and longitude
        bearing_lat, bearing_long = self.random_bearing()
        
        # Define the distance to move along these bearings in meters
        distance = 6 # meters

        # Calculate the new latitude using the random bearing and distance
        geod = Geodesic.WGS84
        new_lat = geod.Direct(self.true_latitude, self.true_longitude, bearing_lat, distance)['lat2']

        # Calculate the new longitude using the random bearing and distance
        new_long = geod.Direct(self.true_latitude, self.true_longitude, bearing_long, distance)['lon2']
        self.offset = self.offset + 0.00000001
        # Update the position
        self.manipulated_longitude = new_long
        self.manipulated_latitude = new_lat
        # self.manipulated_longitude = self.true_longitude + self.offset
        # self.manipulated_latitude = self.true_latitude + self.offset     

        # Create and publish global position message with the updated position
        global_message = NavSatFix()
        global_message.longitude = self.manipulated_longitude
        global_message.latitude = self.manipulated_latitude
        self.global_publisher.publish(global_message)

        ''' Random Spherical Coordinates -- Own Path '''

        # if self.manipulated_longitude is None or self.manipulated_latitude is None:
        #     self.manipulated_longitude = msg.longitude
        #     self.manipulated_latitude = msg.latitude
        
        # # Generate random bearings for latitude and longitude
        # bearing_lat, bearing_long = self.random_bearing()
        
        # # Define the distance to move along these bearings in meters
        # distance = 5  # meters

        # # Calculate the new latitude using the random bearing and distance
        # geod = Geodesic.WGS84
        # new_lat = geod.Direct(self.manipulated_latitude, self.manipulated_longitude, bearing_lat, distance)['lat2']

        # # Calculate the new longitude using the random bearing and distance
        # new_long = geod.Direct(self.manipulated_latitude, self.manipulated_longitude, bearing_long, distance)['lon2']
        
        # # Update the position
        # self.manipulated_longitude = new_long
        # self.manipulated_latitude = new_lat

        # # Create and publish global position message with the updated position
        # global_message = NavSatFix()
        # global_message.longitude = self.manipulated_longitude
        # global_message.latitude = self.manipulated_latitude
        # self.global_publisher.publish(global_message)

        # ''' Random Cartesian Coordinates '''

        # self.true_longitude = msg.longitude
        # self.true_latitude = msg.latitude
        # print(self.true_latitude)
        
        # # Generate random offsets
        # random_offset_long = random.uniform(-0.0004, 0.0004)
        # random_offset_lat = random.uniform(-0.0004, 0.0004)

        # # Manipulate the GPS data
        # self.manipulated_longitude = msg.longitude + random_offset_long
        # self.manipulated_latitude = msg.latitude + random_offset_lat

        # # Create and publish global position message
        # global_message = NavSatFix()
        # global_message.longitude = self.manipulated_longitude
        # global_message.latitude = self.manipulated_latitude
        # self.global_publisher.publish(global_message)

        # # Create and publish local position message ---- subject to change -----
        # local_message = Odometry()
        # local_message.pose.pose.position.x = self.manipulated_longitude
        # local_message.pose.pose.position.y = self.manipulated_latitude
        # self.local_publisher.publish(local_message)

    def log_gps_data(self):
        if self.true_longitude is not None and self.true_latitude is not None:
            self.get_logger().info(f'\nTrue Longitude: {self.true_longitude} \nTrue Latitude: {self.true_latitude}')
        if self.manipulated_longitude is not None and self.manipulated_latitude is not None:
            self.get_logger().info(f'\nManipulated Longitude: {self.manipulated_longitude} \nManipulated Latitude: {self.manipulated_latitude}')

def main(args=None):
    rclpy.init(args=args)
    node = SpoofManipulate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
