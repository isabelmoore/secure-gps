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
#### To use KalmanFilter(backup) change the trackeobject[i].x[1] to trackeobject[i].x[2]
# from colorama import Fore, Style
# trackNames = []
# track = []

# camera1 = []
# camera2 = []
# camera3 = []
# camera4 = []
# radar1 = []
# track1=[]
# track2=[]
# piksi = []
# piksi2 = []

# class Calibration():

#     class Radar:
#         def __init__(self, R=np.zeros((3,3)), X0=np.zeros(3)):
#             self.R = R
#             self.X0 = X0

#     class Camera:
#         def __init__(self, H=np.zeros((3,3)), X0=np.zeros(3), P =np.zeros((3,4))):
#             self.H = H
#             self.X0 = X0
#             self.P = P
            

class Sensor(Node):
    def __init__(self):
        # Variable to store all Kalman Filter objects
        super().__init__('Sensor_Stacker')
        self.start = 0
        self.declare_parameter('infrastructure', False)
        self.infrastructure = self.get_parameter('infrastructure').get_parameter_value().bool_value
        self.get_logger().info("Infrastructure variable: " + str(self.infrastructure) )
        # self.timer = self.create_timer(1.0, self.log_all_data)

        # ### For Sensors on the Infrastructure
        # if self.infrastructure:
        #     self.xmin1_=[0,0,0,0,0,0,0,0,0,0,0,0]
        #     self.xmin2_=[0,0,0,0,0,0,0,0,0,0,0,0]
        #     self.xmin3_=[0,0,0,0,0,0,0,0,0,0,0,0]
        #     self.xmin4_=[0,0,0,0,0,0,0,0,0,0,0,0]
        #     self.ymin1_=[0,0,0,0,0,0,0,0,0,0,0,0]
        #     self.ymin2_=[0,0,0,0,0,0,0,0,0,0,0,0]
        #     self.ymin3_=[0,0,0,0,0,0,0,0,0,0,0,0]
        #     self.ymin4_=[0,0,0,0,0,0,0,0,0,0,0,0]
        
        #     # Sensor Class initialization
        #     self.Radar = Calibration().Radar()
        #     self.Camera = []
        #     self.piksi = []
        #     self.piksi2=[]
        #     self.InfraSensorsData= SensorHCompact()

 
        #     self.getCalibrationParameters()


        #     self.create_subscription(SMSensor,'/radar_Estimate',  self.radar_callback,10)
        #     self.create_subscription( PoseWithCovarianceStamped,'/piksi/enu_pose_fix', self.piksi_callback,10)
        #     self.create_subscription(PoseWithCovarianceStamped,'/piksi2/enu_pose_fix',  self.piksi2_callback,10)
        #     self.create_subscription(BoundingBoxes, 'camera1/bounding_boxes',  self.darknet1_callback,10)
        #     #self.create_subscription(BoundingBoxes, 'camera2/bounding_boxes', self.darknet2_callback,10)
        
        
        #     self.pubSensors = self.create_publisher(SensorHCompact, '/InfraSensorsData', 10)
        #     self.pubcam1 = self.create_publisher(Odometry,'/Camera1',  10)
        #     self.pubcam2 = self.create_publisher(Odometry,'/Camera2', 10)
        #     self.pubcam3 = self.create_publisher(Odometry,'/Camera3',10)
        #     self.pubcam4 = self.create_publisher(Odometry,'/Camera4',10)
        #     self.pubradar = self.create_publisher( PointStamped,'/Radar', 10)
        #     #print('Publishing')
        #     self.prev_time = Clock().now().nanoseconds
        
        #     #self.pubSensors.publish(self.Sensors) 
        #     self.create_timer(1/self.pub_rate, self.Infra_Sensor_publisher)
        

        ### For sensors in the car
        if not self.infrastructure:


            print('yassss')
            self.pub_rate = 20
            self.VehicleSensorsData= SensorHCompact()
            self.create_subscription(Imu,'/vehicle/imu_cartesian', self.imu_callback,10)
            self.create_subscription(NavSatFix, '/vehicle/gps_cartesian', self.gps_callback, 10)
            # self.create_subscription(NavSatFix, '/vehicle/gps_vehicle_cartesian', self.gps_vehicle_callback, 10)
            # self.create_subscription(NavSatFix, '/vehicle/gps_manipulated_cartesian', self.gps_callback, 10)
            # self.create_subscription(Odometry,'/vehicle/odom_cartesian', self.odom_callback,10)
            self.pubVehicleSensors = self.create_publisher(SensorHCompact, '/VehicleSensorsData', 10)
            self.create_timer(1/self.pub_rate, self.Vehicle_Sensor_publisher)
            self.last_position = None  # To store the last GPS position

    

        
	# ####################################################################################
	# ## This function pulls the camera calibration parameters from 				  ######
	# ## camera_calibration_results                                                 ######
	# ####################################################################################
    # def getCalibrationParameters(self):
    #     #print("HI", os.getcwd())
    #     #Reading camera calibration parameters
    #     #cwd = os.path.dirname(os.getcwd()) + r'/Config/camera_calibration_results/Extrinsic/'
    #     cwd = os.getcwd() +r'/src/infraware_ros/Config/camera_calibration_results/Extrinsic/'
    #     camera_extrinsic_filename = ['camera1.yaml', 'camera2.yaml']
    #     # Z Rotation 180 degrees

    #     for filename in camera_extrinsic_filename:
    #         with open(os.path.join(cwd, filename)) as file:
    #             yaml_data = yaml.full_load(file)
    #             file.close()

    #         Rz = np.array([[-1,  0, 0],
    #                        [ 0, -1, 0],
    #                        [ 0,  0, 1]])
    #         K = np.array(yaml_data['K'])
    #         R = np.array(yaml_data['rotation_matrix']).dot(Rz)
    #         X0 = np.array(yaml_data['location']).flatten() 
    #         X0_= np.array(yaml_data['location'])
    #         H = np.linalg.pinv(np.dot(K,R))
    #         P = np.dot(np.dot(K,R),np.hstack((np.eye(3),-X0_)))
    #         self.Camera.append(Calibration().Camera(H, X0, P))

    #     print('Camera parameters loaded!')

    #     #Reading radar calibration parameters
    #     cwd_r = os.getcwd() +r'/src/infraware_ros/Config/radar_calibration_result/'
    #     with open(os.path.join(cwd_r, 'radar.yaml')) as file:
    #             yaml_data_r = yaml.full_load(file)
    #             file.close()

    #     self.Radar.X0 = np.array(yaml_data_r['location']).flatten()
    #     self.Radar.R = np.array(yaml_data_r['rotation_matrix'])
    #     print('Radar parameters loaded!')



	# ####################################################################################
	# ## ROS subscriber for darknet on camera 1                                     ######                                         
	# ####################################################################################
    # def darknet1_callback(self, data):
    #     flag = 0
        
        
    #     points = []
        
    #     distance = [0,0,0,0,0,0,0,0,0,0,0]
    #     camera = SensorH()
    #     camera1 = Odometry()
        
        

    #     self.curr_time = Clock().now().nanoseconds
    #     dt = (self.curr_time - self.prev_time)*1e-9
    #     if dt == 0:
    #          print('bad callback time!')
    #          return
        
    #     self.prev_time = self.curr_time
    #     a = -0.0007726
    #     b = -0.10596
        
        
    #     for i in range(len(data.bounding_boxes)):
            
    #         distance[i]= (data.bounding_boxes[i].xmin - self.xmin1_[i])**2 + (data.bounding_boxes[i].ymin - self.ymin1_[i])**2
            
    #         self.xmin1_[i]= data.bounding_boxes[i].xmin
    #         self.ymin1_[i]= data.bounding_boxes[i].ymin
            
    #         if (data.bounding_boxes[i].xmin >1 and distance[i]>10):
    #             point = self.Camera_Transform(self.Camera[0].X0, self.Camera[0].H,a,b,
    #                                         data.bounding_boxes[i].xmax,
    #                                         data.bounding_boxes[i].xmin,
    #                                         data.bounding_boxes[i].ymax)
    #             #print(point)
    #             #erx = point[0]-self.piksi.x
    #             #ery = point[1]-self.piksi.y
    #             #d = np.sign(point[1]-self.Camera[0].X0[1])*(point[1]-self.Camera[0].X0[1])

    #             #if point[1]<40 and point[1]>0:
                   
    #             points.append(point)
    #             camera.x.append(point[0])
    #             camera.y.append(point[1])
    #             camera.id.append(1)

    #             camera1.pose.pose.position.x = point[0]
    #             camera1.pose.pose.position.y = point[1]
    #             curr_time = Clock().now()
    #             camera1.header.stamp = curr_time.to_msg()
    #             self.pubcam1.publish(camera1)
        
    #     if points:       
    #         self.InfraSensorsData.sensor.append(camera)

                


    #     self.start = 1


	# ####################################################################################
	# ## ROS subscriber for darknet on camera 2                                     ######                                         
	# ##################################################################True
    # def darknet2_callback(self, data):
    #     flag = 0
        
    #     points = []
        

    
    #     distance = [0,0,0,0,0,0,0,0,0,0,0]

    #     self.curr_time = Clock().now().nanoseconds
    #     dt = (self.curr_time - self.prev_time)*1e-9
    #     if dt == 0:
    #         print('bad callback time!')
    #         return
        
    #     self.prev_time = self.curr_time

    #     a= -0.0011319
    #     b=-0.15
    #     b=0
    #     camera = SensorH()
    #     camera2 = Odometry()

    #     for i in range(len(data.bounding_boxes)):
            
    #         camera = SensorH()
    #         distance[i]= (data.bounding_boxes[i].xmin - self.xmin2_[i])**2 + (data.bounding_boxes[i].ymin - self.ymin2_[i])**2
    #         self.xmin2_[i]= data.bounding_boxes[i].xmin
    #         self.ymin2_[i]= data.bounding_boxes[i].ymin

    #         if (data.bounding_boxes[i].xmin >1 and distance[i]>10):
    #             point = self.Camera_Transform(self.Camera[1].X0, self.Camera[1].H,a,b,
    #                                           data.bounding_boxes[i].xmax,
    #                                           data.bounding_boxes[i].xmin,
    #                                           data.bounding_boxes[i].ymax)
    #             points.append(point)
    #             #erx = point[0]-self.piksi.x
    #             #ery = point[1]-self.piksi.y
                
    #             camera.x.append(point[0])
    #             camera.y.append(point[1])
    #             camera.id.append(2)

    #             camera2.pose.pose.position.x = point[0]
    #             camera2.pose.pose.position.y = point[1]
    #             curr_time = Clock().now()
    #             camera2.header.stamp = curr_time.to_msg()
    #             self.pubcam2.publish(camera2)
    #     if points:     
    #             self.InfraSensorsData.sensor.append(camera)
 

    #     self.start=1
                  
        


	# ####################################################################################
	# ## ROS subscriber for darknet on camera 3                                     ######                                         
	# ####################################################################################
    # def darknet3_callback(self, data):
    #     flag = 0
        
    #     points = []

    #     distance = [0,0,0]
    #     camera = SensorH()
    #     camera3 = Odometry()
    #     self.curr_time = Clock().now().nanoseconds
    #     dt = (self.curr_time - self.prev_time)*1e-9
    #     if dt == 0:
    #         print('bad callback time!')
    #         return
        
    #     self.prev_time = self.curr_time

    #     a=-0.001011
    #     b=0.345
    #     b=0.1
        

    #     for i in range(len(data.bounding_boxes)):
            

    #         # Check for camera Freeze
    #         distance[i]= (data.bounding_boxes[i].xmin - self.xmin3_[i])**2 + (data.bounding_boxes[i].ymin - self.ymin3_[i])**2
    #         self.xmin3_[i]= data.bounding_boxes[i].xmin
    #         self.ymin3_[i]= data.bounding_boxes[i].ymin


    #         if (data.bounding_boxes[i].xmin >1 and distance[i]>0):
    #             point = self.Camera_Transform(self.Camera[2].X0, self.Camera[2].H,a,b,
    #                                           data.bounding_boxes[i].xmax,
    #                                           data.bounding_boxes[i].xmin,
    #                                           data.bounding_boxes[i].ymax)
    #             points.append(point)
    #             #erx = point[0]-self.piksi.x
    #             #ery = point[1]-self.piksi.y
                
    #             camera.x.append(point[0])
    #             camera.y.append(point[1])
    #             camera.id.append(3)


    #             camera3.pose.pose.position.x = point[0]
    #             camera3.pose.pose.position.y = point[1]
    #             curr_time = Clock().now()
    #             camera3.header.stamp = curr_time.to_msg()
    #             if np.linalg.norm(point) < 200:
    #                    self.pubcam3.publish(camera3)
                   
    #     if points:
    #         if self.InfraSensorsData.sensor:
    #             if self.InfraSensorsData.sensor[-1]:        
    #                 self.InfraSensorsData.sensor.append(camera)
    #             else:
    #                 self.InfraSensorsData.sensor[-1] = camera          
    #     self.start=1


        
	# ####################################################################################
	# ## ROS subscriber for darknet on camera 4                                     ######
	# ####################################################################################  
    # def darknet4_callback(self, data):
    #     flag = 0
        
    #     points = []
        

    #     distance = [0,0,0]
          
    #     self.curr_time = Clock().now().nanoseconds
    #     dt = (self.curr_time - self.prev_time)*1e-9
    #     if dt == 0:
    #         print('bad callback time!')
    #     #print('\tType: Camera 4')
    #     a=-0.00129
    #     b=0.524
        
        
    #     camera = SensorH()
    #     camera4 = Odometry()

    #     for i in range(len(data.bounding_boxes)):
            
    #         # Check for camera Freeze
            
    #         distance[i]= (data.bounding_boxes[i].xmin - self.xmin4_[i])**2 + (data.bounding_boxes[i].ymin - self.ymin4_[i])**2
            
    #         self.xmin4_[i]= data.bounding_boxes[i].xmin
    #         self.ymin4_[i]= data.bounding_boxes[i].ymin
            


    #         if (data.bounding_boxes[i].xmin >1 and distance[i]>0):
    #             point = self.Camera_Transform(self.Camera[3].X0, self.Camera[3].H,a,b,
    #                                           data.bounding_boxes[i].xmax,
    #                                           data.bounding_boxes[i].xmin,
    #                                           data.bounding_boxes[i].ymax)
    #             points.append(point)
    #             #erx = point[0]-self.piksi.x
    #             #ery = point[1]-self.piksi.y
    #             #self.file4.write(str(erx)+","+str(ery)+","+str(self.piksi.x)+","+str(self.piksi.y)+"\n")
    #             camera.x.append(point[0])
    #             camera.y.append(point[1])
    #             camera.id.append(4)

    #             camera4.pose.pose.position.x = point[0]
    #             camera4.pose.pose.position.y = point[1]
    #             curr_time = Clock().now()
    #             camera4.header.stamp = curr_time.to_msg
    #             self.pubcam4.publish(camera4)
    #     if points:          
    #         if self.InfraSensorsData.sensor:
    #             if self.InfraSensorsData.sensor[-1]:        
    #                 self.InfraSensorsData.sensor.append(camera)
    #             else:
    #                 self.InfraSensorsData.sensor[-1] = camera
            
    #     self.start=1             
	# ####################################################################################
	# ## ROS subscriber for Piksi                                                   ######
	# ####################################################################################
    # def piksi_callback(self, data):

    #     self.piksi = data.pose.pose.position
    #     piksi.append(np.array([self.piksi.x, self.piksi.y]))
    # def piksi2_callback(self, data):
       
    #     self.piksi2 = data.pose.pose.position
    #     piksi2.append(np.array([self.piksi2.x, self.piksi2.y +1.5]))

    # #####################################################################################
    # ## Ros Subscriber for radar data and                                          ######
    # ## transforming them into piksi coordinates                                    ######
    # #####################################################################################
    # def radar_callback(self, data):
        
    #     flag = 1
        
            
    #     radar_data = []
    #     Radar = SensorH()
    #     radar_pub_data = PointStamped()
    #     # Compute time since last message
    #     self.curr_time = Clock().now().nanoseconds
    #     dt = (self.curr_time - self.prev_time)*1e-9
    #     if dt == 0:
    #         print('bad callback time!')
    #         return
    #     self.prev_time = self.curr_time


    #     radar = np.zeros(2)
    #     radarv = np.zeros(2)

    #     curr_time = Clock().now()

    #     #print('Num Radar Pts: ', len(data.x))
    #     for i in range(len(data.x)):
            
    #         radar[0]  = -(data.y[i])
    #         radar[1]  = data.x[i]
    #         radarv[0] = data.vx[i]
    #         radarv[1] = data.vy[i]
 
    #         tmp = self.Radar.X0 + self.Radar.R.dot(radar)
            
    #         radar_data.append(np.array([tmp[0],tmp[1]]))
            
            
                  
    #         Radar.x.append(tmp[0])
    #         Radar.y.append(tmp[1])
    #         Radar.id.append(5)


    #         radar_pub_data.point.x = tmp[0]
    #         radar_pub_data.point.y = tmp[1]
    #         radar_pub_data.header.stamp = curr_time.to_msg()
    #         self.pubradar.publish(radar_pub_data)


    #     self.InfraSensorsData.sensor.append(Radar)  


    #################################################################
    ####### Vehicle Callbacks
 	#######################################################


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

        # if self.last_position is not None:
        #     # Calculate the differences
        #     dx = (current_position[0] - self.last_position[0]) / (n + 1)
        #     dy = (current_position[1] - self.last_position[1]) / (n + 1)

        #     # Append interpolated points
        #     for i in range(1, n + 1):
        #         interpolated_x = self.last_position[0] + dx * i
        #         interpolated_y = self.last_position[1] + dy * i
                
        #         gps_point = SensorH()
        #         gps_point.x = [interpolated_x]
        #         gps_point.y = [interpolated_y]
        #         gps_point.id = [2]  # Assuming a constant ID for simplicity
        #         self.VehicleSensorsData.sensor.append(gps_point)

        # Append the current position as the last point
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


    # def gps_manipulated_callback(self, msg):
    #     gps_manipulated_point = SensorH()
    #     # Assuming the manipulated GPS message has been converted to Cartesian coordinates
    #     gps_manipulated_point.x = [msg.latitude]
    #     gps_manipulated_point.y = [msg.longitude]
    #     # gps_manipulated_point.vx = [msg.vx]  # Assuming velocity is provided
    #     # gps_manipulated_point.vy = [msg.vy]
    #     gps_manipulated_point.id = [2]
        
        # self.VehicleSensorsData.sensor.append(gps_manipulated_point)
    # def odom_callback(self, msg):
    #     odom_point = SensorH()
    #     # Odometry data typically contains both position and velocity
    #     odom_point.x = [msg.pose.pose.position.x]
    #     odom_point.y = [msg.pose.pose.position.y]
    #     # odom_point.vx = [msg.twist.twist.linear.x]
    #     # odom_point.vy = [msg.twist.twist.linear.y]
    #     odom_point.id = [4]
    #     self.VehicleSensorsData.sensor.append(odom_point)


        

    # #####################################################################################
    # ## This function handles darknet transformation into world                     ######
    # #####################################################################################
    # def Camera_Transform(self, X0, H,a,b, xmax, xmin, ymax):

    #     point = np.array([(xmax + xmin)*0.5, ymax, 1])
    #     H1 = H.dot(point)
    #     lamb = (X0[-1]-a*ymax-b) / H1[-1]
  
    #     point =  X0 + lamb * H1

    #     return point[:-1]
    
    def Infra_Sensor_publisher(self):     
        self.pubSensors.publish(self.InfraSensorsData)
        self.InfraSensorsData = SensorHCompact()

    def Vehicle_Sensor_publisher(self):
        self.pubVehicleSensors.publish(self.VehicleSensorsData)
        print("sensor",self.VehicleSensorsData)
        self.VehicleSensorsData = SensorHCompact()
        for sensor_data in self.VehicleSensorsData.sensor:
            self.get_logger().info(f"\nIMU Data:\t \t{sensor_data.x[0]}, \t{sensor_data.y[0]}")
   
    # def log_all_data(self):
    #     current_time = self.get_clock().now().to_msg().sec  # Get current time in seconds
    #     log_message = f"\n---Data Summary at {current_time}---"
        
    #     # Iterate through the sensor data stored in VehicleSensorsData
    #     for sensor_data in self.VehicleSensorsData.sensor:
    #         if sensor_data.id[0] == 1:
    #             log_message += f"\nIMU Data:\t \t{sensor_data.x[0]}, \t{sensor_data.y[0]}"
    #         elif sensor_data.id[0] == 2:
    #             log_message += f"\nGPS Data:\t \t{sensor_data.x[0]}, \t{sensor_data.y[0]}"
    #         # elif sensor_data.id[0] == 3:
    #         #     log_message += f"\nGPS Manipulated Data:\t \t{sensor_data.x[0]}, \t{sensor_data.y[0]}"
    #         elif sensor_data.id[0] == 3:
    #             log_message += f"\nOdometry Data:\t \t{sensor_data.x[0]}, \t{sensor_data.y[0]}"
        
    #     self.get_logger().info(log_message)


def main():
    rclpy.init()
    node =  Sensor()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()


