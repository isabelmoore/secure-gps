from abc import ABC, abstractmethod

from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class ROSVehicleAbstractor(ABC):
    def __init__(self, node_object, vehicle: str, odometry_topic: str):
        # Save node object and vehicle name as class variables
        self.node = node_object
        self.vehicle = vehicle
        
        # State and setpoint dictionaries for vehicle
        self.state = {'x': None, 'y': None, 'yaw': None, 'v': None}
        
        # Flag which is set to true when vehicle state is initialized
        self.got_state = False
        
        # Subscribe to odometry
        self.odom_sub = self.node.create_subscription(Odometry, odometry_topic, self.vehicle_state_callback, 1)

    #########################################
    # Getter for vehicle state
    def get_state(self):
        if self.got_state:
            return self.state
        else:
            return None


    #########################################
    # Update vehicle state with odometry
    def vehicle_state_callback(self, msg: Odometry):
        self.state['x'] = msg.pose.pose.position.x
        self.state['y'] = msg.pose.pose.position.y
        #self.state['v'] = msg.twist.twist.linear.x  # Get velocity from wheel odometry for now
        
        # Convert quaternion to yaw using tf transformations
        quat = msg.pose.pose.orientation
        euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        self.state['yaw'] = euler[2]
        
        if self.state['v'] is not None:
            self.got_state = True


    #########################################
    # Latitude control methods
    @abstractmethod
    def publish_vehicle_lat(self, steering):
        pass
    
    @abstractmethod
    def steering_callback(self, msg):
        pass


    #########################################
    # Longitudinal control methods
    @abstractmethod
    def publish_vehicle_long(self, throttle, brake):
        pass
    
    @abstractmethod
    def throttle_callback(self, msg):
        pass
    
    @abstractmethod
    def brake_callback(self, msg):
        pass

