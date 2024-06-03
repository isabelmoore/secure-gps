#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler as QOE
from tf_transformations import euler_from_quaternion as EFQ
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion, Point, Vector3
from scipy.spatial.transform import Rotation as R
import numpy as np
import matplotlib.pyplot as plt
from .KalmanFilterIMU import KalmanFilterIMU
from vectornav_msgs.msg import CommonGroup
from dbw_ford_msgs.msg import SteeringReport
from swiftnav_ros2_driver.msg import Baseline



np.set_printoptions(suppress=True, precision=2)

class SensorProcessingNode(Node):
    def __init__(self):
        super().__init__('sensor_processing_node')
        # self.imu_subscription = self.create_subscription(Imu, '/vehicle/imu_plugin/out', self.imu_callback, 10)
        self.gps_vehicle_subscription = self.create_subscription(NavSatFix, '/vehicle/gps/fix', self.gps_vehicle_callback, 10)
        self.gps_subscription = self.create_subscription(NavSatFix, '/manipulated_gps/global', self.gps_manipulated_callback, 10) # use local because all cartesian coordinates are local
        # self.odometry_subscription = self.create_subscription(Odometry, '/vehicle/odom', self.odometry_callback, 10)
        
        self.create_subscription(Baseline, '/baseline',  self.piksi_callback,10)
        # self.imu_subscription = self.create_subscription(Imu, '/vectornav/raw', self.imu_callback, 10)
        self.imu_subscription = self.create_subscription(CommonGroup,'/vectornav/raw/common',self.imu_callback,10)
        self.gps_subscription = self.create_subscription(NavSatFix,"/vectornav/gnss", self.gps_callback, 10)
        # self.gps_subscription = self.create_subscription(NavSatFix, '/manipulated_gps/global', self.gps_manipulated_callback, 10) # use local because all cartesian coordinates are local
        self.wheel_subscription = self.create_subscription(SteeringReport,'/vehicle/steering_report', self.wheel_encoder_callback,10)
        # self.odometry_subscription = self.create_subscription(PoseWithCovarianceStamped, '/vectornav/pose', self.odometry_callback, 10)


        
        self.imu_publisher = self.create_publisher(Imu, '/vehicle/imu_cartesian', 10)
        self.gps_publisher = self.create_publisher(NavSatFix, '/vehicle/gps_cartesian', 10)
        self.gps_vehicle_publisher = self.create_publisher(NavSatFix, '/vehicle/gps_vehicle_cartesian', 10)
        self.gps_manipulated_publisher = self.create_publisher(NavSatFix, '/vehicle/gps_manipulated_cartesian', 10)
        # self.odom_publisher = self.create_publisher(Odometry, '/vehicle/odom_cartesian', 10)

        # Initialize variables for data accumulation
        self.imu_position = np.array([0.0, 0.0, 0.0])
        
        self.imu_position_ = np.array([0.0, 0.0, 0.0])
        self.gps_position = np.array([0.0, 0.0, 0.0])
        self.gps_manipulated_position = np.array([0.0, 0.0, 0.0])
        self.odom_position = np.array([0.0, 0.0, 0.0])

        # Initialize variables for IMU processing
        self.last_time = self.get_clock().now()
        self.imu_velocity = np.array([0.0, 0.0, 0.0])
        # self.orientation_q = np.array([0.,0.,0.,0.])
        self.gravity = np.array([0.0, 0.0, 9.81])  # Gravity vector
        self.earth_rate = np.array([2*np.pi/86400,0.0,0.0])

        # Timer to log data every second (change the period as needed)
        self.timer = self.create_timer(0.05, self.log_all_data)

        ''' Position Plotting '''
        self.imu_positions = [[], [], []]  # Separate lists for X, Y, Z
        self.gps_positions = [[], [], []]
        self.gps_manipulated_positions = [[], [], []]
        self.odom_positions = [[], [], []]
        self.GPS_published = False
        self.Imu_published = False
        self.i = 0
        self.offset = 0.1
        self.offset1=0
        self.offset2=0
        self.offset3 =0
        self.vehicle_speed = None
   
        self.initial_gps_position_ecef = None
        self.initial_gps_vehicle_position_ecef = None
        self.initial_gps_manipulated_position_ecef = None
        self.initial_odom = None
        self.piksi_initial = None

        # Properly initialize last_plot_time
        self.last_plot_time = self.get_clock().now()
        self.times = []

        self.imu_track = KalmanFilterIMU()
        self.gps_track = KalmanFilterIMU()

                
        # self.file1 = open("lanechange_spoof_detail.txt","w")
        # self.file1 = open("circle_spoof_detail.txt","w")
        self.file1 = open("straight_spoof_detail.txt","w")
	
    
    # ROS subscriber for Piksi                                                        
    def piksi_callback(self, data):
        if self.piksi_initial is None:
           self.piksi_initial = [data.baseline_e_m,data.baseline_n_m,data.baseline_d_m]
        self.piksi = [data.baseline_e_m-self.piksi_initial[0],data.baseline_n_m-self.piksi_initial[1]]

    def wheel_encoder_callback(self,msg):
        self.steering = msg.steering_wheel_angle
        self.vehicle_speed = msg.speed

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        if self.Imu_published == False:
            self.last_time = current_time
            dt = (current_time - self.last_time).nanoseconds / 1e9

            self.Imu_published = True
            self.imu_track.set_state(np.array([[0],[0]]))

        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9 

        self.orientation_q = np.array([msg.quaternion.x,msg.quaternion.y,msg.quaternion.z,msg.quaternion.w])

        # Convert quaternion to rotation matrix
        rotation = R.from_quat([
            self.orientation_q[0], 
            self.orientation_q[1], 
            self.orientation_q[2], 
            self.orientation_q[3]])

        deltatheta_dvel = np.array([msg.deltatheta_dvel.x,
                                    msg.deltatheta_dvel.y,
                                    msg.deltatheta_dvel.z])
        deltatheta_dvel_world = rotation.apply(deltatheta_dvel)
        deltatheta_dvel_world_c = deltatheta_dvel_world + dt*(self.gravity-np.cross(self.earth_rate,self.imu_velocity))
        self.imu_velocity += deltatheta_dvel_world_c
        self.imu_position_ += np.array(self.imu_velocity * dt + deltatheta_dvel_world_c*dt/2)
   
        self.last_time = current_time

        if self.vehicle_speed is None: 
            self.imu_position = self.imu_position_
        else:
            z = np.array([[self.imu_position_[0]],[self.imu_position_[1]]])

            self.imu_track.EKF_predict(self.vehicle_speed,self.steering,dt)
            self.imu_track.EKF_update(self.vehicle_speed,z)
            self.imu_position[1] = self.imu_track.x[0][0]
            self.imu_position[0] = self.imu_track.x[1][0]
    
        # Assuming self.imu_position contains [x_acceleration, y_acceleration, z_acceleration] data
        imu_msg = Imu()

        # Setting linear acceleration (replace Vector3() with actual data if necessary)
        imu_msg.linear_acceleration = Vector3(x=self.imu_position[0], y=self.imu_position[1], z=self.imu_position[2])
        imu_msg.angular_velocity = Vector3(x=self.imu_velocity[0], y=self.imu_velocity[1], z=self.imu_velocity[2])
        self.imu_publisher.publish(imu_msg)
        # self.gps_vehicle_publisher.publish(gps_msg)
        

    def gps_callback(self, msg):
        # Convert latitude, longitude, and altitude to Cartesian ECEF coordinates
        ecef_coords = self.gps_to_ecef(msg.latitude, msg.longitude, msg.altitude)
        # Make sure to convert the tuple to a NumPy array
        if self.initial_gps_position_ecef is None:
            self.initial_gps_position_ecef = np.array(ecef_coords)
            self.initial_gps_rotation = self.rotation_matrix(msg.latitude, msg.longitude, msg.altitude)
            #self.gps_published = True

        self.gps_position = self.initial_gps_rotation.dot(np.array(ecef_coords) - self.initial_gps_position_ecef)
        # self.gps_position = np.array(ecef_coords) - self.initial_gps_position_ecef
        # if self.gps_position[1]>5 and self.gps_position[1]<15:
        self.offset1 += 0.025
        self.offset2 += 0.05
        self.offset3 += 0.1
        
        gps_msg = NavSatFix()
        gps_msg.latitude = self.gps_position[0] #+self.offset3
        gps_msg.longitude = self.gps_position[1]
        gps_msg.altitude = self.gps_position[2]
        self.gps_offest1 = self.gps_position[0]+self.offset1
        self.gps_offest2 = self.gps_position[0]+self.offset2
        self.gps_offest3 = self.gps_position[0]+self.offset3

        # Publish the manipulated GPS data
        self.gps_publisher.publish(gps_msg)
        self.i += 1
        if self.i >5:
            self.GPS_published = True
        if self.piksi_initial is not None:
           self.file1.write(str(self.piksi[1]) +"," + str(self.piksi[0]) +"," + str(self.gps_position[1]) + "," +str(self.gps_position[0]) +"," +str(self.gps_offest1)+","+ str(self.gps_offest2)+ ","+str(self.gps_offest3)+"\n")

    def gps_manipulated_callback(self, msg):
        # Convert latitude, longitude, and altitude to Cartesian ECEF coordinates
        ecef_coords = self.gps_to_ecef(msg.latitude, msg.longitude, msg.altitude)
        # Make sure to convert the tuple to a NumPy array
        if self.initial_gps_manipulated_position_ecef is None:
            self.initial_gps_manipulated_position_ecef = np.array(ecef_coords)
            self.initial_gps_manipulated_rotation = self.rotation_matrix(msg.latitude, msg.longitude, msg.altitude)
            #self.gps_published = True

        self.gps_manipulated_position = self.initial_gps_manipulated_rotation.dot(np.array(ecef_coords) - self.initial_gps_manipulated_position_ecef)
        # self.gps_position = np.array(ecef_coords) - self.initial_gps_position_ecef
        # if self.gps_position[1]>5 and self.gps_position[1]<15:
        
        gps_msg = NavSatFix()
        gps_msg.latitude = self.gps_manipulated_position[0]
        gps_msg.longitude = self.gps_manipulated_position[1]
        gps_msg.altitude = self.gps_manipulated_position[2]
  

        # Publish the manipulated GPS data
        self.gps_manipulated_publisher.publish(gps_msg)
        # self.i += 1
        # if self.i >5:
        #     self.GPS_published = True
        # if self.piksi_initial is not None:
        #    self.file1.write(str(self.piksi[1]) +"," + str(self.piksi[0]) +"," + str(self.gps_position[1]) + "," +str(self.gps_position[0]) +"," +str(self.gps_offest1)+","+ str(self.gps_offest2)+ ","+str(self.gps_offest3)+"\n")


    def gps_vehicle_callback(self, msg):
        # Convert latitude, longitude, and altitude to Cartesian ECEF coordinates
        ecef_coords = self.gps_to_ecef(msg.latitude, msg.longitude, msg.altitude)
        # Make sure to convert the tuple to a NumPy array
        if self.initial_gps_vehicle_position_ecef is None:
            self.initial_gps_vehicle_position_ecef = np.array(ecef_coords)
            self.initial_gps_vehicle_rotation = self.rotation_matrix(msg.latitude, msg.longitude, msg.altitude)
            #self.gps_published = True
            # self.gps_track.set_state(np.array([[0],[0]]))

        self.gps_vehicle_position = self.initial_gps_vehicle_rotation.dot(np.array(ecef_coords) - self.initial_gps_vehicle_position_ecef)
        # self.gps_vehicle_position = np.array(ecef_coords) - self.initial_gps_vehicle_position_ecef

        gps_msg = NavSatFix()

        gps_msg.latitude = self.gps_vehicle_position[0]
        gps_msg.longitude = self.gps_vehicle_position[1]
            
        self.gps_vehicle_publisher.publish(gps_msg)

    def gps_to_ecef(self, latitude, longitude, altitude):

        # WGS 84 ellipsoid parameters
        a = 6378137.0
        f = 1 / 298.257223563
        e_sq = f * (2 - f)
        b= 6356752.3
        e_sq = 1-(b**2/a**2)

        # Convert latitude, longitude, and altitude from degrees to radians
        lat_rad = latitude/180*np.pi
        lon_rad = longitude/180*np.pi

        # Radius of curvature in the prime vertical
        N = a / np.sqrt(1 - e_sq * np.sin(lat_rad)**2)

        # Calculate ECEF coordinates
        X = (N + altitude) * np.cos(lat_rad) * np.cos(lon_rad)
        Y = (N + altitude) * np.cos(lat_rad) * np.sin(lon_rad)
        Z = (N * (1 - e_sq) + altitude) * np.sin(lat_rad)
        
        return (X, Y, Z) 
    def rotation_matrix(self,latitude,longitude,altitude):
        lat_rad = latitude/180*np.pi
        lon_rad = longitude/180*np.pi
        sin_lat = np.sin(lat_rad)
        cos_lat = np.cos(lat_rad)
        sin_lon = np.sin(lon_rad)
        cos_lon = np.cos(lon_rad)
        Rot = np.array([[-sin_lon, cos_lon,0],[-sin_lat*cos_lon,-sin_lat*sin_lon,cos_lat],[cos_lat*cos_lon,-cos_lat*sin_lon,sin_lat]])
        return Rot
    # def odometry_callback(self, msg):
    #     # Extract the position and convert to a NumPy array
    #     position = msg.pose.pose.position
    #     # velocity = msg.twist.twist.linear
    #     if self.initial_odom is None:
    #         self.initial_odom = np.array([position.x, position.y, position.z])
    #     if self.initial_gps_rotation is not None:    
    #        self.odom_position = self.initial_gps_rotation.dot(np.array([position.x, position.y, position.z]) - self.initial_odom)
          
    #        odom_msg = Odometry()
    #        # self.odom_velocity = np.array([velocity.x, velocity.y, velocity.z])

    #        # Set frame ID if necessary (example shown)
    #        odom_msg.header.frame_id = "odom"
    #        odom_msg.child_frame_id = "base_link"

    #        odom_msg.pose.pose.position = Point(x=self.odom_position[0], y=self.odom_position[1], z=self.odom_position[2])
    #        # odom_msg.twist.twist.linear = Vector3(x=msg.twist.twist.linear.x, y= msg.twist.twist.linear.y,z=msg.twist.twist.linear.z)
    #        # odom_msg.twist.twist.angular= Vector3(x=msg.twist.twist.angular.z,y=msg.twist.twist.angular.y,z=msg.twist.twist.angular.z)

    #        self.odom_publisher.publish(odom_msg)

    def log_all_data(self):
        current_time = self.get_clock().now().to_msg().sec  # Get current time in seconds
        current_time = self.get_clock().now()
        dt = (current_time - self.last_plot_time).nanoseconds / 1e9  # Time since last plot update in seconds
        if dt < 1.0:  # Ensure at least 1 second has passed
            return
        self.last_plot_time = current_time
        # Update the time series data
        self.times.append(current_time.nanoseconds / 1e9)  # Convert to seconds for plotting

        log_message = "\n---Data Summary:---"
        if self.imu_position is not None:
            log_message += f"\nIMU Cartesian: {self.imu_position}"
        if self.gps_position is not None:
            log_message += f"\nGPS Cartesian: {self.gps_position}"
        if self.gps_position is not None:
            log_message += f"\nGPS Manipulated Cartesian: {self.gps_manipulated_position}"
        if self.odom_position is not None:
            log_message += f"\nOdom Cartesian: {', '.join(f'{x:.2f}' for x in self.odom_position)}"
        self.get_logger().info(log_message)

        # log_message = "\n---Data Summary:---"
        # if self.imu_velocity is not None:
        #     log_message += f"\nIMU Velocity: {self.imu_velocity}"
        # if self.odom_velocity is not None:
        #     log_message += f"\nOdom Velocity: {', '.join(f'{x:.2f}' for x in self.odom_velocity)}"
        self.get_logger().info(log_message)
        
        ''' Position Plotting '''
        for i in range(3):  # Update stored data for all three axes
            self.imu_positions[i].append(self.imu_position[i])
            self.gps_positions[i].append(self.gps_position[i])
            self.gps_manipulated_positions[i].append(self.gps_manipulated_position[i])
            self.odom_positions[i].append(self.odom_position[i])


def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
