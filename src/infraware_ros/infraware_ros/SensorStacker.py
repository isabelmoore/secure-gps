#!usr/bin/env python3
import numpy as np
import yaml
import os
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu

# from darknet_ros_msgs.msg import BoundingBoxes
from vision_msgs.msg import SMSensor, SensorH , SensorHCompact

class Sensor(Node):
    def __init__(self):
        # Variable to store all Kalman Filter objects
        super().__init__('Sensor_Stacker')
        self.start = 0
        self.declare_parameter('infrastructure', False)
        self.infrastructure = self.get_parameter('infrastructure').get_parameter_value().bool_value
        self.get_logger().info("Infrastructure variable: " + str(self.infrastructure) )

        ### For sensors in the car
        if not self.infrastructure:
            print('yassss')
            self.pub_rate = 20
            self.VehicleSensorsData= SensorHCompact()
            self.create_subscription(Imu,'/vehicle/imu_cartesian', self.imu_callback,10)
            # self.create_subscription(NavSatFix, '/vehicle/gps_cartesian', self.gps_callback, 10)
            # self.create_subscription(NavSatFix, '/vehicle/gps_vehicle_cartesian', self.gps_vehicle_callback, 10)
            self.create_subscription(NavSatFix, '/vehicle/gps_manipulated_cartesian', self.gps_callback, 10)
            # self.create_subscription(Odometry,'/vehicle/odom_cartesian', self.odom_callback,10)
            self.pubVehicleSensors = self.create_publisher(SensorHCompact, '/VehicleSensorsData', 10)
            self.create_timer(1/self.pub_rate, self.Vehicle_Sensor_publisher)
            self.last_position = None  # To store the last GPS position


    def imu_callback(self,msg):
        imu_point = SensorH()
        # Assuming the IMU message contains orientation and linear acceleration
        imu_point.x = [msg.linear_acceleration.x]
        imu_point.y = [msg.linear_acceleration.y]
        # imu_point.vx = [msg.angular_velocity.x]  # Using angular velocity as an example
        # imu_point.vy = [msg.angular_velocity.y]
        imu_point.id = [1]
        self.VehicleSensorsData.sensor.append(imu_point)

    def gps_callback(self, msg):
        current_position = (msg.latitude, msg.longitude)
        n = 50  # Number of points to interpolate

        gps_point = SensorH()
        gps_point.x = [current_position[0]]
        gps_point.y = [current_position[1]]
        gps_point.id = [2]  # Assuming a constant ID for simplicity
        self.VehicleSensorsData.sensor.append(gps_point)

        # Update the last position to the current position
        self.last_position = current_position
    def gps_vehicle_callback(self, msg):
        current_position = (msg.latitude, msg.longitude)
        # Append the current position as the last point
        gps_point = SensorH()
        gps_point.x = [current_position[0]]
        gps_point.y = [current_position[1]]
        gps_point.id = [3]  # Assuming a constant ID for simplicity
        self.VehicleSensorsData.sensor.append(gps_point)
    
    def Infra_Sensor_publisher(self):     
        self.pubSensors.publish(self.InfraSensorsData)
        self.InfraSensorsData = SensorHCompact()

    def Vehicle_Sensor_publisher(self):
        self.pubVehicleSensors.publish(self.VehicleSensorsData)
        print("sensor",self.VehicleSensorsData)
        self.VehicleSensorsData = SensorHCompact()
        for sensor_data in self.VehicleSensorsData.sensor:
            self.get_logger().info(f"\nIMU Data:\t \t{sensor_data.x[0]}, \t{sensor_data.y[0]}")


def main():
    rclpy.init()
    node =  Sensor()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()


