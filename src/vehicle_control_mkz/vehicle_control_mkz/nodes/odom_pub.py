#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from vehicle_control_mkz.algorithm.pid import PID
from vehicle_control_mkz.vehicle_interface.mkz_interface import ROSInterfaceMKZ
from tf_transformations import quaternion_from_euler as QOE
from tf_transformations import euler_from_quaternion as EFQ
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node 

# This Node publishes the vehicle odometry and publishes an accumulated path message to RVIZ
# It also enables for switching between simulation and real vehicle odometry

class OdometryPublisher(Node):
    def __init__(self):
        # Initialize the node
        super().__init__('odom_pub_node')
        
        # Load the parameters in case you want to use them elsewhere
        self.declare_parameter('simulation', True)
        self.sim = self.get_parameter('simulation').get_parameter_value().bool_value
        self.get_logger().info("Simulation: " + str(self.sim))
        
        # Get path rate publish parameter
        self.declare_parameter('path_publish_rate', 10.0)
        path_publish_rate = self.get_parameter('path_publish_rate').get_parameter_value().double_value
        self.get_logger().info("Path Publish Rate: " + str(path_publish_rate) + " hz")
        
        # Get output frame ID parameter
        self.declare_parameter('frame_id', 'world')
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.get_logger().info("Frame ID: " + self.frame_id)

        #FOR SIM ONLY: Sensor topics 
        if self.sim:
            # Subscribe to the odometry topic for the vehicle location
            self.odom_sub = self.create_subscription(Odometry,"/vehicle/ground_truth_odom",self.odom_callback, 1)
        
        # For Real MKZ: Sensor Topics 
        if not self.sim:
            # TODO: Subscribe to the GPS topic for the vehicle location
            pass

        # Publishers
        self.odom_pub = self.create_publisher(Odometry,'/vehicle/odom1',1)
        self.path_display_pub = self.create_publisher(Path,'/vehicle/travelled_path',1)
   
        # [x, y, yaw] just for display purposes
        self.veh_pose = [0.0, 0.0, 0.0]
        
        # Create timer to publish at the specified rate
        self.timer = self.create_timer(1/path_publish_rate, self.publish_path)
        
        # Define global output message variables
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        
        # Initialize Odometry recieved flag to false
        self.odom_flag = False
        
        
    def odom_callback(self, msg: Odometry):    
        # Create PoseStamped from Odometry message
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
            
        # Update the timestamp and append the pose to the path message
        self.path_msg.header.stamp = msg.header.stamp # Update the timestamp
        self.path_msg.poses.append(pose)
            
        # Republish the odometry message
        self.odom_pub.publish(msg)
        
        # If this is the first time we've recieved an odometry message, print a message
        if not self.odom_flag:
            self.odom_flag = True
            self.get_logger().info("Recieved First Odometry Message")
        
    def publish_path(self):
        # Only publish if we have gotten our first odometry message
        if self.odom_flag: 
            self.path_display_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    op = OdometryPublisher()
    
    # Spin for callbacks
    rclpy.spin(op)
    
    # Destroy the node after Ctrl+C
    op.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
