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
        # self.gps_subscription = self.create_subscription(NavSatFix, '/manipulated_gps/global', self.gps_manipulated_callback, 10) # use local because all cartesian coordinates are local
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
        # self.gps_manipulated_publisher = self.create_publisher(NavSatFix, '/vehicle/gps_manipulated_cartesian', 10)
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
        # self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8))
        # self.fig.suptitle('Sensor Data Over Time')
        # self.axs[0].set_title('X Position')
        # self.axs[1].set_title('Y Position')
        # self.axs[2].set_title('Z Position')

        # for ax in self.axs:
        #     ax.set_xlabel('Time')
        #     ax.set_ylabel('Position')
        #     ax.grid(True)

        # # For 3D trajectory plot
        # self.fig_3d = plt.figure(figsize=(10, 7))
        # self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        # self.ax_3d.set_title('3D Trajectories of IMU, GPS, and Odom Data')

        # # Setting the labels and title
        # self.ax_3d.set_xlabel('X Position')
        # self.ax_3d.set_ylabel('Y Position')
        # self.ax_3d.set_zlabel('Z Position')
        # self.ax_3d.legend()
        

        ''' Velocity Plotting '''
        # self.imu_velocities = [[], []]
        # self.odom_velocities = [[], []]

        # self.fig, self.axs = plt.subplots(2, 1, figsize=(10, 8))
        # self.fig.suptitle('Sensor Data Over Time')
        # self.axs[0].set_title('Linear X Velocity')
        # self.axs[1].set_title('Linear Y Velocity')

        # for ax in self.axs:
        #     ax.set_xlabel('Time')
        #     ax.set_ylabel('Linear Velocity')
        #     ax.grid(True)

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
        # self.piksi_append[0].append(data.baseline_e_m-self.piksi_initial[0])
        # self.piksi_append[1].append(data.baseline_n_m-self.piksi_initial[1])     


    def wheel_encoder_callback(self,msg):
        self.steering = msg.steering_wheel_angle
        self.vehicle_speed = msg.speed

    def imu_callback(self, msg):
        current_time = self.get_clock().now()
        if self.Imu_published == False:
            self.last_time = current_time
            dt = (current_time - self.last_time).nanoseconds / 1e9
            # self.orientation_q = np.array([msg.quaternion.x,msg.quaternion.y,msg.quaternion.z,msg.quaternion.w])
            # self.orientation = np.array([msg.yawpitchroll.x,msg.yawpitchroll.y,msg.yawpitchroll.z])
            self.Imu_published = True
            self.imu_track.set_state(np.array([[0],[0]]))

        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9 
            # print( "else", self.last_time, current_time,dt)
            # delta_theta = np.array([msg.deltatheta_dtheta.x,msg.deltatheta_dtheta.y,msg.deltatheta_dtheta.z])
            # self.orientation = np.array([msg.yawpitchroll.x,msg.yawpitchroll.y,msg.yawpitchroll.z])
            # self.orientation += delta_theta

            # gamma = np.linalg.norm(delta_theta)/2
            # if gamma >= 0.00001:
            #     Psai = (np.sin(gamma)/(2*gamma))*delta_theta
            # else:
            #     Psai = 0.5*delta_theta
            # Psai = delta_theta
            # print("gamma",gamma, delta_theta,Psai)
           
            
            # Psai_matrix = np.array([[0,-Psai[2],Psai[1]],[Psai[2],0,-Psai[0]],[-Psai[1],Psai[0],0]])
            # tf_11 = (np.eye(3)-Psai_matrix).reshape(3,3)
            # Quaternion_prediction_1 = np.append(tf_11,Psai.reshape(3,1),axis=1)
        
            # Quaternion_prediction_2 = np.array([[-Psai[0],-Psai[1],-Psai[2],1]])
            # Quaternion_prediction = np.append(Quaternion_prediction_1,Quaternion_prediction_2,axis=0).reshape(4,4)
            # # print("Quaternion_prediction:",Quaternion_prediction_1,Quaternion_prediction_2,Quaternion_prediction)
            # orientation_q = Quaternion_prediction.dot(self.orientation_q)
            # self.orientation_q = orientation_q/np.linalg.norm(orientation_q)

     
        # Extract linear acceleration and orientation from the message
        # linear_acceleration = np.array([
        #     msg.linear_acceleration.x, 
        #     msg.linear_acceleration.y, 
        #     msg.linear_acceleration.z])
        
        # self.orientation_q = msg.orientation
        self.orientation_q = np.array([msg.quaternion.x,msg.quaternion.y,msg.quaternion.z,msg.quaternion.w])



        # Convert quaternion to rotation matrix
        rotation = R.from_quat([
            self.orientation_q[0], 
            self.orientation_q[1], 
            self.orientation_q[2], 
            self.orientation_q[3]])
        # rotation = R.from_euler('zyx', [self.orientation[0], self.orientation[1], self.orientation[2]], degrees=True)
        # print(R)
        # self.gravity = np.array([0.0, msg.linear_acceleration.y, 9.81]) 

        # Rotate acceleration from IMU frame to world frame
        # linear_acceleration_world = rotation.apply(linear_acceleration) - self.gravity
        # linear_acceleration_world = rotation.apply(linear_acceleration - self.gravity)

        # Integrate acceleration to get velocity
        # self.imu_velocity += linear_acceleration_world * dt
        # self.imu_velocity += linear_acceleration * dt
        deltatheta_dvel = np.array([msg.deltatheta_dvel.x,
                                    msg.deltatheta_dvel.y,
                                    msg.deltatheta_dvel.z])
        deltatheta_dvel_world = rotation.apply(deltatheta_dvel)
        deltatheta_dvel_world_c = deltatheta_dvel_world + dt*(self.gravity-np.cross(self.earth_rate,self.imu_velocity))
        self.imu_velocity += deltatheta_dvel_world_c
        

        # Integrate velocity to get position. This needs to accumulate over time, not just the last interval
        #self.imu_position += np.array(self.imu_velocity * dt )
        self.imu_position_ += np.array(self.imu_velocity * dt + deltatheta_dvel_world_c*dt/2)
        # print(deltatheta_dvel_world,deltatheta_dvel_world_c*dt/2)
        # self.imu_position += np.array([0.02,0.01,0])
        # self.imu_position += np.array([-0.0006,0.0007,0])
        # Update last time for next callback
        self.last_time = current_time

        if self.vehicle_speed is None: 
            self.imu_position = self.imu_position_
        else:
            z = np.array([[self.imu_position_[0]],[self.imu_position_[1]]])

            # self.gps_track.EKF_predict(self.vehicle_speed,self.steering,dt)
            # gps_msg = NavSatFix()
            # gps_msg.latitude = self.gps_track.x[0][0]
            # gps_msg.longitude = self.gps_track.x[1][0]

            self.imu_track.EKF_predict(self.vehicle_speed,self.steering,dt)
            self.imu_track.EKF_update(self.vehicle_speed,z)
            self.imu_position[0] = self.imu_track.x[0][0]
            self.imu_position[1] = self.imu_track.x[1][0]
            
            # self.imu_position = self.imu_position_


        # Assuming self.imu_position contains [x_acceleration, y_acceleration, z_acceleration] data
        imu_msg = Imu()

        # Setting linear acceleration (replace Vector3() with actual data if necessary)
        imu_msg.linear_acceleration = Vector3(x=self.imu_position[1], y=self.imu_position[0], z=self.imu_position[2])
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
        gps_msg.latitude = self.gps_position[0] +self.offset3
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
        # if self.vehicle_speed is None:
        #     pass
        # else:
        #     z = np.array([[self.gps_vehicle_position[0]],[self.gps_vehicle_position[1]]])
        #     # self.gps_track.EKF_predict(self.vehicle_speed,self.steering,1)
        #     self.gps_track.EKF_update(self.vehicle_speed,z)
        #     self.offset += 0.01
        #     gps_msg = NavSatFix()
        #     gps_msg.latitude = self.gps_track.x[0][0]
        #     gps_msg.longitude = self.gps_track.x[1][0]
            
        self.gps_vehicle_publisher.publish(gps_msg)


    # def gps_manipulated_callback(self, msg):
    #     # Convert latitude, longitude, and altitude to Cartesian ECEF coordinates
    #     ecef_coords = self.gps_to_ecef(msg.latitude, msg.longitude, msg.altitude)
    #     # Make sure to convert the tuple to a NumPy array
    #     if self.initial_gps_manipulated_position_ecef is None:
    #         self.initial_gps_manipulated_position_ecef = np.array(ecef_coords)
    #         self.initial_gps_manipulated_rotation = self.rotation_matrix(msg.latitude, msg.longitude, msg.altitude)
            
    #         self.gps_published = True
    #     self.gps_manipulated_position = self.initial_gps_manipulated_rotation.dot(np.array(ecef_coords) - self.initial_gps_manipulated_position_ecef)

    #     gps_manipulated_msg = NavSatFix()
    #     gps_manipulated_msg.latitude = self.gps_manipulated_position[0]
    #     gps_manipulated_msg.longitude = self.gps_manipulated_position[1]
    #     gps_manipulated_msg.altitude = self.gps_manipulated_position[2]

    #     # Publish the manipulated GPS data
    #     self.gps_manipulated_publisher.publish(gps_manipulated_msg)

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
        Rot = np.array([[-sin_lon, cos_lon,0],[-sin_lat*cos_lon,-sin_lat*sin_lon,cos_lat],[-sin_lon, cos_lon,0],[cos_lat*cos_lon,-cos_lat*sin_lon,sin_lat]])
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

        # Update plots
        # for i, ax in enumerate(self.axs):
        #     ax.clear()  # Clear existing data
        #     ax.plot(self.times, self.imu_positions[i], label='IMU')
        #     ax.plot(self.times, self.gps_positions[i], label='GPS')
        #     ax.plot(self.times, self.gps_manipulated_positions[i], label='GPS Manipulated')
        #     ax.plot(self.times, self.odom_positions[i], label='Odom')
        #     ax.legend()
        #     ax.grid(True)
        #     ax.set_ylim([-150, 150])
        #     ax.set_xlabel('Time')
        #     ax.set_ylabel('Position')
        #     ax.set_title(['X Position', 'Y Position', 'Z Position'][i])
        
        ''' Velocity Plotting '''
        # for i in range(2):  # Update stored data for all three axes
        #     self.imu_velocities[i].append(self.imu_velocity[i])
        #     self.odom_velocities[i].append(self.odom_velocity[i])

        # # Update plots
        # for i, ax in enumerate(self.axs):
        #     ax.clear()  # Clear existing data
        #     ax.plot(self.times, self.imu_velocities[i], label='IMU')
        #     ax.plot(self.times, self.odom_velocities[i], label='Odom')
        #     ax.legend()
        #     ax.grid(True)
        #     ax.set_ylim([-20, 20])
        #     ax.set_xlabel('Time')
        #     ax.set_ylabel('Position')
        #     ax.set_title(['Linear X Velocity', 'Linear Y Velocity'][i])

        # plt.tight_layout()
        # plt.pause(0.001)  # Pause briefly to update the plot
        
        # # Update 3D plot separately
        # self.ax_3d.clear()
        # self.ax_3d.plot(self.imu_positions[0], self.imu_positions[1], self.imu_positions[2], label='IMU')
        # self.ax_3d.plot(self.gps_positions[0], self.gps_positions[1], self.gps_positions[2], label='GPS')
        # self.ax_3d.plot(self.gps_manipulated_positions[0], self.gps_manipulated_positions[1], self.gps_manipulated_positions[2], label='GPS')
        # self.ax_3d.plot(self.odom_positions[0], self.odom_positions[1], self.odom_positions[2], label='Odom')
        
        # self.ax_3d.set_xlim([-250, 250])
        # self.ax_3d.set_ylim([-250, 250])
        # self.ax_3d.set_zlim([-250, 250])
        # self.ax_3d.set_xlabel('X Position')
        # self.ax_3d.set_ylabel('Y Position')
        # self.ax_3d.set_zlabel('Z Position')
        # self.ax_3d.set_title('3D Trajectories of IMU, GPS, and Odom Data')
        # self.ax_3d.legend()
        

        # plt.tight_layout()
        # plt.pause(0.001)  # Pause briefly to update the plot
        
def main(args=None):
    rclpy.init(args=args)
    node = SensorProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import NavSatFix
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from tf_transformations import quaternion_from_euler as QOE
# from tf_transformations import euler_from_quaternion as EFQ
# from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion, Point, Vector3, Twist


# from scipy.spatial.transform import Rotation as R
# import random
# import numpy as np
# import matplotlib.pyplot as plt
# from threading import Thread
# import csv
# import os
# np.set_printoptions(suppress=True, precision=2)


# class SensorProcessingNode(Node):
#     def __init__(self):
#         super().__init__('sensor_processing_node')
#         self.imu_subscription = self.create_subscription(Imu, '/vehicle/imu_plugin/out', self.imu_callback, 10)
#         self.gps_subscription = self.create_subscription(NavSatFix, '/vehicle/fix', self.gps_callback, 10)
#         self.gps_subscription = self.create_subscription(NavSatFix, '/manipulated_gps/global', self.gps_manipulated_callback, 10) # use local because all cartesian coordinates are local
#         self.odometry_subscription = self.create_subscription(Odometry, '/vehicle/odom', self.odometry_callback, 10)
        
#         self.imu_publisher = self.create_publisher(Imu, '/vehicle/imu_cartesian', 10)
#         self.gps_publisher = self.create_publisher(NavSatFix, '/vehicle/gps_cartesian', 10)
#         self.gps_manipulated_publisher = self.create_publisher(NavSatFix, '/vehicle/gps_manipulated_cartesian', 10)
#         self.odom_publisher = self.create_publisher(Odometry, '/vehicle/odom_cartesian', 10)

#         # Initialize variables for data accumulation
#         self.imu_position = np.array([0.0, 0.0, 0.0])
#         self.gps_position = np.array([0.0, 0.0, 0.0])
#         self.gps_manipulated_position = np.array([0.0, 0.0, 0.0])
#         self.odom_position = np.array([0.0, 0.0, 0.0])

#         # Initialize variables for IMU processing
#         self.last_time = self.get_clock().now()
#         self.imu_velocity = np.array([0.0, 0.0, 0.0])
#         self.gravity = np.array([0.0, 0.0, 9.81])  # Gravity vector

#         # Timer to log data every second (change the period as needed)
#         self.timer = self.create_timer(1.0, self.log_all_data)
        
#         ''' Position Plotting '''
#         self.imu_positions = [[], [], []]  # Separate lists for X, Y, Z
#         self.gps_positions = [[], [], []]
#         self.gps_manipulated_positions = [[], [], []]
#         self.odom_positions = [[], [], []]

#         self.fig, self.axs = plt.subplots(3, 1, figsize=(10, 8))
#         self.fig.suptitle('Sensor Data Over Time')
#         self.axs[0].set_title('X Position')
#         self.axs[1].set_title('Y Position')
#         self.axs[2].set_title('Z Position')

#         for ax in self.axs:
#             ax.set_xlabel('Time')
#             ax.set_ylabel('Position')
#             ax.grid(True)

#         # # For 3D trajectory plot
#         # self.fig_3d = plt.figure(figsize=(10, 7))
#         # self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
#         # self.ax_3d.set_title('3D Trajectories of IMU, GPS, and Odom Data')

#         # # Setting the labels and title
#         # self.ax_3d.set_xlabel('X Position')
#         # self.ax_3d.set_ylabel('Y Position')
#         # self.ax_3d.set_zlabel('Z Position')
#         # self.ax_3d.legend()
        

#         ''' Velocity Plotting '''
#         self.imu_velocities = [[], []]
#         self.odom_velocities = [[], []]

#         # self.fig, self.axs = plt.subplots(2, 1, figsize=(10, 8))
#         # self.fig.suptitle('Sensor Data Over Time')
#         # self.axs[0].set_title('Linear X Velocity')
#         # self.axs[1].set_title('Linear Y Velocity')

#         # for ax in self.axs:
#         #     ax.set_xlabel('Time')
#         #     ax.set_ylabel('Linear Velocity')
#         #     ax.grid(True)

#         self.initial_gps_position_ecef = None
#         self.initial_gps_manipulated_position_ecef = None
#         self.initial_odom = None

#         # Properly initialize last_plot_time
#         self.last_plot_time = self.get_clock().now()
#         self.times = []

#     def imu_callback(self, msg):
#         current_time = self.get_clock().now()
#         if self.last_time is None:
#             self.last_time = current_time
#             return  # Skip the first callback to initialize last_time

#         dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds

#         # Extract linear acceleration and orientation from the message
#         linear_acceleration = np.array([
#             msg.linear_acceleration.x, 
#             msg.linear_acceleration.y, 
#             msg.linear_acceleration.z])
        
#         orientation_q = msg.orientation

#         # Convert quaternion to rotation matrix
#         rotation = R.from_quat([
#             orientation_q.x, 
#             orientation_q.y, 
#             orientation_q.z, 
#             orientation_q.w])

#         # Rotate acceleration from IMU frame to world frame
#         linear_acceleration_world = rotation.apply(linear_acceleration) - self.gravity

#         # Integrate acceleration to get velocity
#         self.imu_velocity += linear_acceleration_world * dt

#         # Integrate velocity to get position. This needs to accumulate over time, not just the last interval
#         self.imu_position += np.array(self.imu_velocity * dt)

#         # Update last time for next callback
#         self.last_time = current_time

#         # Assuming self.imu_position contains [x_acceleration, y_acceleration, z_acceleration] data
#         imu_msg = Imu()

#         # Setting linear acceleration (replace Vector3() with actual data if necessary)
#         imu_msg.linear_acceleration = Vector3(x=self.imu_position[0], y=self.imu_position[1], z=self.imu_position[2])
#         imu_msg.angular_velocity = Vector3(x=self.imu_velocity[0], y=self.imu_velocity[1], z=self.imu_velocity[2])
#         self.imu_publisher.publish(imu_msg)


#     def gps_callback(self, msg):
#         # Convert latitude, longitude, and altitude to Cartesian ECEF coordinates
#         ecef_coords = self.gps_to_ecef(msg.latitude, msg.longitude, msg.altitude)
#         # Make sure to convert the tuple to a NumPy array
#         if self.initial_gps_position_ecef is None:
#             self.initial_gps_position_ecef = np.array(ecef_coords)
#         self.gps_position = np.array(ecef_coords) - self.initial_gps_position_ecef
        
#         gps_msg = NavSatFix()
#         gps_msg.latitude = self.gps_position[0]
#         gps_msg.longitude = self.gps_position[1]
#         gps_msg.altitude = self.gps_position[2]

#         # Publish the manipulated GPS data
#         self.gps_publisher.publish(gps_msg)

#     def gps_manipulated_callback(self, msg):
#         # Convert latitude, longitude, and altitude to Cartesian ECEF coordinates
#         ecef_coords = self.gps_to_ecef(msg.latitude, msg.longitude, msg.altitude)
#         # Make sure to convert the tuple to a NumPy array
#         if self.initial_gps_manipulated_position_ecef is None:
#             self.initial_gps_manipulated_position_ecef = np.array(ecef_coords)
#         self.gps_manipulated_position = np.array(ecef_coords) - self.initial_gps_manipulated_position_ecef

#         gps_manipulated_msg = NavSatFix()
#         gps_manipulated_msg.latitude = self.gps_manipulated_position[0]
#         gps_manipulated_msg.longitude = self.gps_manipulated_position[1]
#         gps_manipulated_msg.altitude = self.gps_manipulated_position[2]

#         # Publish the manipulated GPS data
#         self.gps_manipulated_publisher.publish(gps_manipulated_msg)

#     def gps_to_ecef(self, latitude, longitude, altitude):

#         # WGS 84 ellipsoid parameters
#         a = 6378137.0
#         f = 1 / 298.257223563
#         e_sq = f * (2 - f)

#         # Convert latitude, longitude, and altitude from degrees to radians
#         lat_rad = np.radians(latitude)
#         lon_rad = np.radians(longitude)

#         # Radius of curvature in the prime vertical
#         N = a / np.sqrt(1 - e_sq * np.sin(lat_rad)**2)

#         # Calculate ECEF coordinates
#         X = (N + altitude) * np.cos(lat_rad) * np.cos(lon_rad)
#         Y = (N + altitude) * np.cos(lat_rad) * np.sin(lon_rad)
#         Z = (N * (1 - e_sq) + altitude) * np.sin(lat_rad)
        
#         return (X, Y, Z)
        
#     def odometry_callback(self, msg):
#         # Extract the position and convert to a NumPy array
#         position = msg.pose.pose.position
#         if self.initial_odom is None:
#             self.initial_odom = np.array([position.x, position.y, position.z])
            
#         self.odom_position = np.array([position.x, position.y, position.z]) - self.initial_odom

#         velocity = msg.twist.twist.linear
#         self.odom_velocity = np.array([velocity.x, velocity.y, velocity.z])
        
#         odom_msg = Odometry()

#         # Set frame ID if necessary (example shown)
#         odom_msg.header.frame_id = "odom"
#         odom_msg.child_frame_id = "base_link"

#         odom_msg.pose.pose.position = Point(x=self.odom_position[0], y=self.odom_position[1], z=self.odom_position[2])
#         odom_msg.twist.twist.linear = Vector3(x=self.odom_velocity[0], y=self.odom_velocity[1], z=self.odom_velocity[2])
#         self.odom_publisher.publish(odom_msg)

#     def log_all_data(self):
#         current_time = self.get_clock().now().to_msg().sec  # Get current time in seconds
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_plot_time).nanoseconds / 1e9  # Time since last plot update in seconds
#         if dt < 1.0:  # Ensure at least 1 second has passed
#             return
#         self.last_plot_time = current_time
#         # Update the time series data
#         self.times.append(current_time.nanoseconds / 1e9)  # Convert to seconds for plotting

#         log_message = "\n---Data Summary:---"
#         if self.imu_position is not None:
#             log_message += f"\nIMU Cartesian: {self.imu_position}"
#         if self.gps_position is not None:
#             log_message += f"\nGPS Cartesian: {self.gps_position}"
#         if self.gps_position is not None:
#             log_message += f"\nGPS Manipulated Cartesian: {self.gps_manipulated_position}"
#         if self.odom_position is not None:
#             log_message += f"\nOdom Cartesian: {', '.join(f'{x:.2f}' for x in self.odom_position)}"
#         self.get_logger().info(log_message)

#         # # log_message = "\n---Data Summary:---"
#         # if self.imu_velocity is not None:
#         #     log_message += f"\nIMU Velocity: {self.imu_velocity}"
#         # if self.odom_velocity is not None:
#         #     log_message += f"\nOdom Velocity: {', '.join(f'{x:.2f}' for x in self.odom_velocity)}"
#         # self.get_logger().info(log_message)
        
#         ''' Position Plotting '''
#         for i in range(3):  # Update stored data for all three axes
#             self.imu_positions[i].append(self.imu_position[i])
#             self.gps_positions[i].append(self.gps_position[i])
#             self.gps_manipulated_positions[i].append(self.gps_manipulated_position[i])
#             self.odom_positions[i].append(self.odom_position[i])

#         # Update plots
#         # for i, ax in enumerate(self.axs):
#         #     ax.clear()  # Clear existing data
#         #     ax.plot(self.times, self.imu_positions[i], label='IMU')
#         #     ax.plot(self.times, self.gps_positions[i], label='GPS')
#         #     ax.plot(self.times, self.gps_manipulated_positions[i], label='GPS Manipulated')
#         #     ax.plot(self.times, self.odom_positions[i], label='Odom')
#         #     ax.legend()
#         #     ax.grid(True)
#         #     ax.set_ylim([-150, 150])
#         #     ax.set_xlabel('Time')
#         #     ax.set_ylabel('Position')
#         #     ax.set_title(['X Position', 'Y Position', 'Z Position'][i])
        
#         ''' Velocity Plotting '''
#         # for i in range(2):  # Update stored data for all three axes
#         #     self.imu_velocities[i].append(self.imu_velocity[i])
#         #     self.odom_velocities[i].append(self.odom_velocity[i])

#         # # Update plots
#         # for i, ax in enumerate(self.axs):
#         #     ax.clear()  # Clear existing data
#         #     ax.plot(self.times, self.imu_velocities[i], label='IMU')
#         #     ax.plot(self.times, self.odom_velocities[i], label='Odom')
#         #     ax.legend()
#         #     ax.grid(True)
#         #     ax.set_ylim([-20, 20])
#         #     ax.set_xlabel('Time')
#         #     ax.set_ylabel('Position')
#         #     ax.set_title(['Linear X Velocity', 'Linear Y Velocity'][i])

#         plt.tight_layout()
#         plt.pause(0.001)  # Pause briefly to update the plot
        
#         # # Update 3D plot separately
#         # self.ax_3d.clear()
#         # self.ax_3d.plot(self.imu_positions[0], self.imu_positions[1], self.imu_positions[2], label='IMU')
#         # self.ax_3d.plot(self.gps_positions[0], self.gps_positions[1], self.gps_positions[2], label='GPS')
#         # self.ax_3d.plot(self.gps_manipulated_positions[0], self.gps_manipulated_positions[1], self.gps_manipulated_positions[2], label='GPS')
#         # self.ax_3d.plot(self.odom_positions[0], self.odom_positions[1], self.odom_positions[2], label='Odom')
        
#         # self.ax_3d.set_xlim([-250, 250])
#         # self.ax_3d.set_ylim([-250, 250])
#         # self.ax_3d.set_zlim([-250, 250])
#         # self.ax_3d.set_xlabel('X Position')
#         # self.ax_3d.set_ylabel('Y Position')
#         # self.ax_3d.set_zlabel('Z Position')
#         # self.ax_3d.set_title('3D Trajectories of IMU, GPS, and Odom Data')
#         # self.ax_3d.legend()
        

#         plt.tight_layout()
#         plt.pause(0.001)  # Pause briefly to update the plot