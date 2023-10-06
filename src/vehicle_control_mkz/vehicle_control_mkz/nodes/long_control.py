#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Imports from this package
from vehicle_control_mkz.vehicle_interface.mkz_interface import ROSInterfaceMKZ
from vehicle_control_mkz.algorithm.pid import PID
from vehicle_control_mkz.utility.low_pass_filter import LinearFilter
from vehicle_control_mkz.utility.shared_functions import sat_values

# ROS Message imports
from geometry_msgs.msg import Twist

# Python packages
import yaml

class LongController(Node):
    def __init__(self):
        # Initialize node
        super().__init__('long_control_node')
  
        # Get parameter for simulation or real vehicle
        self.declare_parameter('sim', True)
        self.sim = self.get_parameter('sim').value
        self.get_logger().info("Simulation: " + str(self.sim))
  
        # Get parameter for control rate
        self.declare_parameter('control_rate', 50.0)
        control_rate = self.get_parameter('control_rate').value
        self.get_logger().info("Rate: " + str(control_rate) + " hz")
  
        # Get parameter for vehicle file
        self.declare_parameter('vehicle_yaml_path', 'None')
        vehicle_yaml_path = self.get_parameter('vehicle_yaml_path').value
        self.get_logger().info("Vehicle Yaml Path: " + str(vehicle_yaml_path))
        
        # Get parameter for steering forwarding (used for teleop)
        self.declare_parameter('forward_steering', False)
        self.forward_steering = self.get_parameter('forward_steering').value
        self.get_logger().info("Forward Steering: " + str(self.forward_steering))
  
        # Check that vehicle yaml file was specified
        if vehicle_yaml_path == 'None':
            raise ValueError("No vehicle yaml file specified, send as ros2 parameter 'vehicle_yaml_path'")
        
        # Read YAML file
        with open(vehicle_yaml_path,'r') as stream:
            data_loaded = yaml.safe_load(stream)
   
        # Get parameters from YAML file		
          #### Large error throttle
        self.p1 = data_loaded['PID1']['P']
        self.i1 = data_loaded['PID1']['I']
        self.d1 = data_loaded['PID1']['D']
        #### Large Error Brake
        self.p2 = data_loaded['PID2']['P']
        self.i2 = data_loaded['PID2']['I']
        self.d2 = data_loaded['PID2']['D']
        #### Small Error Brake
        self.p3 = data_loaded['PID3']['P']
        self.i3 = data_loaded['PID3']['I']
        self.d3 = data_loaded['PID3']['D']
        self.error_tolerance = data_loaded['Tol']
        #### For teleop (forward steering)
        self.steer_lim_upper = data_loaded['steerLim']['upper']
        self.steer_lim_lower = data_loaded['steerLim']['lower']
        
        # Format into lists for ease of use
        self.pids_throttle = [self.p1,self.i1,self.d1]
        self.pids_large_brake = [self.p2,self.i2,self.d2]
        self.pids_small_brake = [self.p3,self.i3,self.d3]

        # Initialize PID and filter objects
        self.pid = PID()
        self.pid.set_lims(0.0, 1000.)
        #self.vel_filter = LinearFilter(0.05,[1,-0.95])
        self.throttle_filter = LinearFilter(0.05,[1,-0.95])
        self.brake_filter = LinearFilter(0.05,[1,-0.95])
        # TODO: figure out or remove hardcoded values above
  
        # Desired steering angle when forward steering is enabled
        self.steering_desired = None
        
        # Velocity desired from the lateral controller
        self.vx_desired = None
        
        # Initilize vehicle interface (main publishers and subscribers for vehicle control)
        # This object allows contains the vehicle state and allows for publishing control commands
        if not self.forward_steering: # If not forwarding steering, initialize with just longitudinal control
            self.vehicle = ROSInterfaceMKZ(self, longitudinal_control=True) # TODO: make this not hardcoded to MKZ
        else: # If forwarding steering, initialize with both lateral and longitudinal control
            self.vehicle = ROSInterfaceMKZ(self, lateral_control=True, longitudinal_control=True) # TODO: make this not hardcoded to MKZ
        
        # Subscribe to twist setpoints from the lateral controller
        self.create_subscription(Twist, '/vehicle/twist_cmd', self.twist_callback, 1)
  
        # Create the timer which determines control loop rate (setpoints are updated asynchronously)
        self.timer = self.create_timer(1/control_rate, self.control_loop)


    def control_loop(self):
        # Ensure that vehicle state and setpoint have been received before continuing
        if not self.vehicle.got_state:
            # Send message to every one second
            self.get_logger().info("Waiting for odometry", throttle_duration_sec=1)
            return
        
        # Ensure that setpoint has been received before continuing
        if self.vx_desired is None:
            # Send message to every one second
            self.get_logger().info("Waiting for setpoint", throttle_duration_sec=1)
            return

        # Update setpoint with vehicle object and set error for pid controller
        vx_error = self.vx_desired - self.vehicle.state['v']
        self.pid.update(vx_error)
        self.get_logger().info("Velocity Setpoint: " + str(round(self.vx_desired)) + " Velocity: " + str(round(self.vehicle.state['v'], 1)), throttle_duration_sec=2)
  
        # Throttle block
        if vx_error >= -1.0:   
            self.pid.set_gains(self.pids_throttle[0] * vx_error, self.pids_throttle[1], self.pids_throttle[2] * vx_error)
            u = self.pid.compute_control()      		# Compute Control input
            u = sat_values(u, 0.0, 1.0)      # Bound control input

            # Update throttle filter and set brake to 0
            throttle_cmd = self.throttle_filter.update_filter(u)
            brake_cmd = 0.0
            
            # self.get_logger().info("Throttle block, Throttle: " + str(throttle_cmd) + " Brake: " + str(brake_cmd), throttle_duration_sec=1)

        # Large Brake block
        elif vx_error < -1.0:
            if abs(vx_error) >= self.error_tolerance:
                self.pid.set_gains(self.pids_large_brake[0] * vx_error, self.pids_large_brake[1], self.pids_large_brake[2] * vx_error)
                u = self.pid.compute_control()      # Compute Control input
                u = sat_values(u, 0.0, 1.0)           # Bound control input

                # Update brake filter and set throttle to 0
                brake_cmd = self.brake_filter.update_filter(u)
                throttle_cmd = 0.0
                
                # self.get_logger().info("Large Brake block, Throttle: " + str(throttle_cmd) + " Brake: " + str(brake_cmd), throttle_duration_sec=1)
    
        # Small Brake block
            else:
                self.pid.set_gains(self.pids_small_brake[0], self.pids_small_brake[1], self.pids_small_brake[2])
                u = self.pid.compute_control()           # Compute Control input
                u = sat_values(abs(u), 0.0, 1.0)           # Bound control input

                # Update brake filter and set throttle to 0
                brake_cmd = self.brake_filter.update_filter(u)
                throttle_cmd = 0.0
                
                # self.get_logger().info("Small Brake block, Throttle: " + str(throttle_cmd) + " Brake: " + str(brake_cmd), throttle_duration_sec=1)
    
        # Publish the control commands
        self.vehicle.publish_vehicle_long(throttle_cmd, brake_cmd)
        
        # Forward steering if enabled and steering setpoint is available
        if self.forward_steering and self.steering_desired is not None:
            self.vehicle.publish_vehicle_lat(self.steering_desired)
        
    def twist_callback(self, msg: Twist):
        self.vx_desired = msg.linear.x
        
        # If forwarding steering, publish the steering command
        if self.forward_steering:
            self.steering_desired = sat_values(msg.angular.z, self.steer_lim_lower, self.steer_lim_upper)

    
def main(args=None):
    rclpy.init()
    lc = LongController()
    rclpy.spin(lc)
    lc.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()