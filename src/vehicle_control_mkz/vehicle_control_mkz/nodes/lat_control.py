#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Imports from this package
from vehicle_control_mkz.algorithm.steering_methods import SteeringMethods
from vehicle_control_mkz.vehicle_interface.mkz_interface import ROSInterfaceMKZ
from vehicle_control_mkz.utility.shared_functions import sat_values

# ROS Message imports 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

# Python packages
import yaml


class LatController(Node):
    def __init__(self):
    
        # Initialize node
        super().__init__('lat_control_node')
  
        # Get parameter for simulation or real vehicle
        self.declare_parameter('sim', True)
        self.sim = self.get_parameter('sim').value
        self.get_logger().info("Simulation: " + str(self.sim))
  
        # Get parameter for control rate
        self.declare_parameter('control_rate', 50.0)
        control_rate = self.get_parameter('control_rate').value
        self.get_logger().info("Rate: " + str(control_rate) + " hz")
  
        # Get parameter for desired path publish rate
        self.declare_parameter('path_publish_rate', 1.0)
        path_publish_rate = self.get_parameter('path_publish_rate').value
        self.get_logger().info("Path Rate: " + str(path_publish_rate) + " hz")
  
        # Get parameter for desired speed
        self.declare_parameter('desired_speed', 6.0)
        self.vx_desired = float(self.get_parameter('desired_speed').value)
        self.get_logger().info("Desired Speed: " + str(self.vx_desired) + " m/s")
  
        # Get parameter for path file
        self.declare_parameter('waypoints_file_path', 'None')
        waypoints_file_path = self.get_parameter('waypoints_file_path').value
        self.get_logger().info("Path File: " + str(waypoints_file_path))
  
        # Get parameter for vehicle file
        self.declare_parameter('vehicle_yaml_path', 'None')
        vehicle_yaml_path = self.get_parameter('vehicle_yaml_path').value
        self.get_logger().info("Vehicle Yaml Path: " + str(vehicle_yaml_path))
  
  
        # Check that vehicle yaml file was specified
        if vehicle_yaml_path == 'None':
            raise ValueError("No vehicle yaml file specified, send as ros2 parameter 'vehicle_yaml_path'")
        
        # Read YAML file
        with open(vehicle_yaml_path,'r') as stream:
            data_loaded = yaml.safe_load(stream)
  
        # Get parameters from YAML file
        #### Basic parameters
        look_ahead = data_loaded['lookAhead']
        wheelbase = data_loaded['wheelBase']
        steering_ratio = data_loaded['steeringRatio']
        self.steer_lim_upper = data_loaded['steerLim']['upper']
        self.steer_lim_lower = data_loaded['steerLim']['lower']
        #### Parameters for adaptive velocity and adaptive lookahead
        self.lookahead_min = data_loaded['Adaptive']['lMin']
        self.lookahead_max = data_loaded['Adaptive']['lMax']
        self.vel_min = data_loaded['Adaptive']['vMin']
        self.vel_max = data_loaded['Adaptive']['vMax']
        self.gamma = data_loaded['Adaptive']['gamma']
        self.ay_lim = data_loaded['Adaptive']['ayLim']
  
        # Initialize vehicle interface (main publishers and subscribers for vehicle control)
        # This object allows contains the vehicle state and allows for publishing control commands
        self.vehicle = ROSInterfaceMKZ(self, lateral_control=True) # TODO: make this not hardcoded to MKZ
  
        # Check if path file is valid
        try:
            open(waypoints_file_path, 'r')
        except FileNotFoundError:
            self.get_logger().error("Path file not found")
            waypoints_file_path = None

        # Create new SteeringMethods object for path following algorithms
        if waypoints_file_path is not None:
            self.steering = SteeringMethods(look_ahead, wheelbase, steering_ratio, waypoints_file_path)
            
        # Otherwise if no path file is specified, wait for path on topic
        else:
            self.steering = SteeringMethods(look_ahead, wheelbase, steering_ratio)
        
        # Twist publisher for longitudinal controller
        self.twist_publisher = self.create_publisher(Twist, '/vehicle/twist_cmd', 1)
        
        # Publishers for desired path and target point RVIZ displays
        self.path_publisher = self.create_publisher(Path, '/vehicle/desired_path', 1)
        self.target_point_publisher = self.create_publisher(Marker, '/vehicle/target_point', 1)
        
        # Twist message for longitudinal controller
        self.twist_msg = Twist()
        
        # Path message for path RVIZ display
        self.path_msg = Path()
        self.path_msg.header.frame_id = "world"
        
        # Intermediate PoseStamped message, for path message
        self.pose_msg = PoseStamped()
        self.pose_msg.header.frame_id = "world"
        
        # Target point display message
        self.tp_marker_msg = Marker()
        self.tp_marker_msg.header.frame_id = "world"
        self.tp_marker_msg.type = Marker.SPHERE

        # Create timers for control loop and desired path publishing
        self.control_loop_timer = self.create_timer(1/control_rate, self.control_loop)
        self.path_publish_timer = self.create_timer(1/path_publish_rate, self.publish_path)


    def control_loop(self):
        # Check if state is available
        if not self.vehicle.got_state:
            # Send a message every second
            self.get_logger().info("Waiting for odometry", throttle_duration_sec=1)
        
        # Check if path is available
        elif not self.steering.got_path:
            # Send a message every second
            self.get_logger().info("Waiting for desired path", throttle_duration_sec=1)
        
        # Otherwise since we have a state and setpoint, calculate control and publish
        else:
            # Get vehicle state and setpoint
            state = self.vehicle.get_state()

            # Update lookahead distance for pure pursuit (currently velocity + 1)
            self.steering.update_lookahead(state['x'])
            
            steercmd, curv, absolute_bearing, relative_bearing, target_point = (
                self.steering.method_pure_pursuit(state['x'], state['y'], state['yaw']))
            
            # Calculate desired velocity for longitudinal controller
            vel_cmd = self.steering.method_adaptive_velocity(self.vel_min, self.vel_max, self.ay_lim, curv)
            
            # Limit steer angle for vehicle
            steercmd_final = sat_values(steercmd, self.steer_lim_lower, self.steer_lim_upper)
            
            # Publish twist to longitudinal controller
            self.twist_msg.linear.x = float(vel_cmd)
            self.twist_msg.angular.z = 1/curv * vel_cmd  # unused but helpful for debugging
            self.twist_publisher.publish(self.twist_msg)
            
            # Publish steering command
            self.vehicle.publish_vehicle_lat(steercmd_final)
            
            # Publish target point for RVIZ display
            self.tp_marker_msg.pose.position.x = self.steering.path_array[target_point][0]
            self.tp_marker_msg.pose.position.y = self.steering.path_array[target_point][1]
            self.target_point_publisher.publish(self.tp_marker_msg)
            

    # If path is available publish topic to RVIZ for display
    def publish_path(self):
        # If path is available
        if self.steering.got_path:
            # Loop through and add points to path message
            for point in self.steering.path_array:
                self.pose_msg.pose.position.x = float(point[0])
                self.pose_msg.pose.position.y = float(point[1])
                self.path_msg.poses.append(self.pose_msg)
                
            # Publish path message
            self.path_publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    latc= LatController()
    rclpy.spin(latc)
    latc.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()

