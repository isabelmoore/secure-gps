import rclpy
from dbw_ford_msgs.msg import SteeringCmd, ThrottleCmd, BrakeCmd
from geometry_msgs.msg import TwistStamped
from vehicle_control_mkz.vehicle_interface.vehicle_abstract import ROSVehicleAbstractor

class ROSInterfaceMKZ(ROSVehicleAbstractor):

    def __init__(self, node_object, vehicle='MKZ', odometry_topic='/vehicle/odom1',
                 longitudinal_control=False, lateral_control=False):
        # Initialize parent class (ROSVehicleAbstractor)
        super().__init__(node_object, vehicle, odometry_topic)
        
        # Subscribed to for both lateral and longitudinal control
        self.twist_sub = self.node.create_subscription(TwistStamped, '/vehicle/twist', self.twist_callback, 1)
        
        # If doing Lateral Control
        if lateral_control:
            # Publishers
            self.steer_pub = self.node.create_publisher(SteeringCmd, '/vehicle/steering_cmd', 1)
            
            # Subscribers
            self.steering_sub = self.node.create_subscription(SteeringCmd, '/vehicle/steering_feedback', self.steering_callback, 1)
            
            # Output messages and setup
            self.steering_msg = SteeringCmd()
            self.steering_msg.enable = True
            
        # If doing Longitudinal Control
        if longitudinal_control:
            # Publishers
            self.throttle_pub = self.node.create_publisher(ThrottleCmd, '/vehicle/throttle_cmd', 1)
            self.brake_pub = self.node.create_publisher(BrakeCmd, '/vehicle/brake_cmd', 1)
            
            # Subscribers
            self.throttle_sub = self.node.create_subscription(ThrottleCmd, '/vehicle/throttle_feedback', self.throttle_callback, 1)
            self.brake_sub = self.node.create_subscription(BrakeCmd, '/vehicle/brake_feedback', self.brake_callback, 1)
            
            # Output messages and setup
            self.throttle_msg = ThrottleCmd()
            self.throttle_msg.enable = True
            self.throttle_msg.pedal_cmd_type = 2
            
            self.brake_msg = BrakeCmd()
            self.brake_msg.enable = True
            self.brake_msg.pedal_cmd_type = 2
            
        if not longitudinal_control and not lateral_control:
            self.node.get_logger().info("No control selected method selected, no pubs or subs created")


    #########################################
    # Latitude control methods
    def publish_vehicle_lat(self, steering):
        self.steering_msg.steering_wheel_angle_cmd = steering
        self.steer_pub.publish(self.steering_msg)  # Publish steering command

    def steering_callback(self, msg: SteeringCmd):
        # Process steering feedback if necessary
        pass

    #########################################
    # Longitudinal control methods
    def publish_vehicle_long(self, throttle, brake):
        self.throttle_msg.pedal_cmd = throttle
        self.brake_msg.pedal_cmd = brake
        
        self.throttle_pub.publish(self.throttle_msg)  # Publish throttle command
        self.brake_pub.publish(self.brake_msg)  # Publish brake command

    # This is the wheel odometry topic for the MKZ, for now we will overwrite the velocity with this value
    def twist_callback(self, msg: TwistStamped):
        self.state['v'] = msg.twist.linear.x # Get velocity from wheel odometry for now

    def throttle_callback(self, msg: ThrottleCmd):
        # Process throttle feedback if necessary
        pass

    def brake_callback(self, msg: BrakeCmd):
        # Process brake feedback if necessary
        pass

