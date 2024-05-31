#!usr/bin/env python
import numpy as np
from copy import deepcopy
import pandas as pd
from scipy.optimize import linear_sum_assignment, linprog
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from vision_msgs.msg import SensorH , SensorHCompact, Tracker, TrackerArray
from swiftnav_ros2_driver.msg import Baseline# from swiftnav_ros2_driver.msg import Baseline
import matplotlib.pyplot as plt
from .KalmanFilter import KalmanFilter
from .HealthMonitor import HealthMonitor
# import cvxpy as cp

track1=[[],[],[],[],[],[],[],[]]
Health=[[],[]]
class GNN_Filter_KF_NO(Node):
    def __init__(self):
        super().__init__('FilterInfraKFNO')

        #Initialize variables
        self.start = 0
        self.currID = 0
        self.piksi = []
        self.piksi2=[]
        self.start = 0
        self.counter=[]
        self.finished_cycle = True
        self.Only_radar = [0,0]
        self.trackedObjects = []
        self.numTrackedObjects = 0
        self.mahalanobisDistance = np.array([])
        self.threshold = 1000 #This will effect the new track creation 
        self.numMeasurements = 0
        self.assignmentTable = np.array([])
        self.frameNumber = 0
        self.piksiSmooth=0.000
        self.piksiSmooth2= 0.000
        self.track1 = [[],[],[],[]]
        self.Filter = TrackerArray()
        self.trackedHealth = np.empty((5,2), dtype = object)
        for i in range(3):
            for j in range(1):
                self.trackedHealth[i][j] = HealthMonitor(self.get_logger())
        #publish filtered tracks
        self.pubFilter = self.create_publisher(Tracker,'infraware/filter_vehicle', 10)
        self.color_arr =  plt.cm.cool(np.linspace(0, 1, 5))

        #subscribe to gps and sensor data in cartesian coordiantes
        self.create_subscription(Baseline, '/baseline',  self.piksi_callback,10)
        self.create_subscription(SensorHCompact,'/VehicleSensorsData',  self.Sensor_callback,10)

        print('Filtering!')
        self.prev_time = Clock().now().nanoseconds
        # self.plot_timer = self.create_timer(40, self.export_health_data)  
        self.plot_timer = self.create_timer(1, self.export_pos)

        #Initials plots
        self.fig,self.ax = plt.subplots()
        self.ax.set_xlabel('X(m)')
        self.ax.set_ylabel('Y(m)')
        self.sensor_x = []
        self.sensor_y = []
        self.sensor_x_imu = []
        self.sensor_y_imu = []
        self.sensor_x_gps = []
        self.sensor_y_gps = []
        self.sensor_x_odom = []
        self.sensor_y_odom = []
        self.sensor_x_gps_vehicle = []
        self.sensor_y_gps_vehicle = []
        self.sensor_x_error = []
        self.sensor_y_error = []
        self.piksi_append =[[],[]]
        self.piksi_initial = None
        self.File1=[]
        self.df = []
        self.gps_x =[0.]
        self.gps_y =[0.]
        # self.file1 = open("Track_straight_spoof_KFNO.txt","w")
        # self.file2 = open("Health_straight_spoof_KFNO.txt","w")

        # self.file1 = open("Track_circle_spoof_KFNO.txt","w")
        # self.file2 = open("Health_circle_spoof_KFNO.txt","w")

        # self.file1 = open("Track_lanechange_spoof_KFNO.txt","w")
        # self.file2 = open("Health_lanechange_spoof_KFNO.txt","w")

        # self.file1 = open("Track_figureeight_spoof.txt","w")
        # self.file2 = open("Health_figureeight_spoof.txt","w")
        

    def export_health_data(self):
        for i in range(3):
            for j in range(1):
                self.trackedHealth[i][j].export_to_excel()

    def export_pos(self):
        # Create a DataFrame from the health data, Δm data, and time steps
        # self.get_logger().info(f'odom length: {len(self.sensor_x_gps)}, {len(self.sensor_y_imu)}')
        # self.get_logger().info(f'filter length: {len(self.piksi_append[0])}, {len(self.track1[3])}')

        df = pd.DataFrame(self.df,columns=['Filter X',
            'Filter Y',
            'IMU X',
            'IMU Y',
            'GPS X',
            'GPS Y',
            'Ground Truth X',
            'Ground Truth Y',
            'Algorithm',
            'Spoofing Rate'])
        
        # Export the DataFrame to an Excel file
        # df.to_csv('Track_Sensor_KFNO_Straight.txt', index=False)
        # df.to_csv('Track_Sensor_KFNO_LaneChange.txt', index=False)
        df.to_csv('Track_Sensor_KFNO_Circle.txt', index=False)

	#######################################################
	# ROS subscriber for Piksi                                                        
    def piksi_callback(self, data):
        if self.piksi_initial is None:
           self.piksi_initial = [data.baseline_e_m,data.baseline_n_m,data.baseline_d_m]
        self.piksi = [data.baseline_e_m-self.piksi_initial[0],data.baseline_n_m-self.piksi_initial[1]]

        self.piksi_append[0].append(data.baseline_e_m-self.piksi_initial[0])
        self.piksi_append[1].append(data.baseline_n_m-self.piksi_initial[1])
    # #######################################################
    ### Sensor Subscriber
    def Sensor_callback(self,data):
        dt = 0.05
        n = len(self.trackedObjects)
        m = len(data.sensor) 
        # print("new measurement arrived!")
        # print("\t No of Track is {} and No of measurement is {} \n".format(n,m))
        '''
        #With Radar
        for i in range(m): #measurement
               if len(data.sensor[i].id) ==1:
                   
                   if data.sensor[i].id != 5:
                       self.start = 1
                       break

               else:
                   if data.sensor[i].id[0] != 5:
                       self.start = 1
                       break
        '''
        self.i = 0
        #Without Radar:
        self.start = 1
        if self.start == 1:
           self.predictTrack(dt)
           self.trackedAuxillary = []
           self.trackedAuxillary = np.empty((n,m),dtype= object)
           for k in range(n):
               for j in range(m):  
                   self.trackedAuxillary[k][j] = deepcopy(self.trackedObjects[k])
                   #print("after predict",self.trackedAuxillary[k][j].Update_flag)
                #    print("\n x after predict of track {} and measuremnt {} is : {}".format(k,j,self.trackedAuxillary[k][j].x))
           
           for i in range(m): #measurement
                if data.sensor[i].id:
                    sensor_id = list(data.sensor[i].id)[0]
                    # print(data.sensor[i].x, data.sensor[i].y)
                    if sensor_id == 1:
                        self.sensor_x_imu.append(data.sensor[i].x)
                        self.sensor_y_imu.append(data.sensor[i].y)
                        self.imu_x = data.sensor[i].x
                        self.imu_y = data.sensor[i].y
                    if sensor_id == 2:
                        self.sensor_x_gps.append(data.sensor[i].x)
                        self.sensor_y_gps.append(data.sensor[i].y)
                        self.gps_x = data.sensor[i].x
                        self.gps_y = data.sensor[i].y
                    if sensor_id == 3:
                        self.sensor_x_gps_vehicle.append(data.sensor[i].x)
                        self.sensor_y_gps_vehicle.append(data.sensor[i].y)
                    # if sensor_id == 4:
                    #     self.sensor_x_odom.append(data.sensor[i].x)
                    #     self.sensor_y_odom.append(data.sensor[i].y)
                    # print(data.sensor[i].x, data.sensor[i].y)
                    self.sensor_x.append(data.sensor[i].x)
                    self.sensor_y.append(data.sensor[i].y)
                    #print("data length is {} and i is {}".format(len(data.sensor),i))
                    if len(data.sensor[i].id) == 1:
                        flag__ = np.array([data.sensor[i].id])
                        flag_ = (flag__[0]).item()
                    else:
                        flag__ = np.array([data.sensor[i].id[0]])
                        flag_ = (flag__[0]).item()
                    if flag_ == 1:
                        flag = 2 #Vectornav IMU
                    elif flag_ == 2: #Vectornav GPS
                        flag = 3 
                    elif flag_ == 3: #Low res GPS
                        flag = 4
                    # elif flag_ == 3: #Odom
                    #     flag = 4 
                    # print("the flag is :{}".format(flag))
                    self.i = i
                    self.get_mahalonobis(data.sensor[i],flag,flag_)

           for p in range(n):#tracked objects 
                self.x_size = np.size(self.trackedObjects[p].x)
                Sumx_KF = np.zeros((self.x_size,1), dtype = float)
                Sump_KF = np.zeros((self.x_size,self.x_size), dtype = float)
                Sumx_CI = np.zeros((self.x_size,1), dtype = float)
                Sump_CI = np.zeros((self.x_size,self.x_size), dtype = float)
                C = 0.00
                true_counter = 0
                q_= []
                Omega_ = []
                for q in range(m): #measurement 
                    
                        # print("\n Updated is {} for track {} and measurement {} ".format(self.trackedAuxillary[p][q].Update_flag,p,q))
                        if self.trackedAuxillary[p][q].Update_flag == True:
                    
                            true_counter += 1
                            f = self.trackedAuxillary[p][q].flag_ - 1
                            c = self.trackedHealth[f][p].x[0]
                            ## SHARF
                            q_.append(c*self.trackedAuxillary[p][q].q_)
                            Omega_.append(c*self.trackedAuxillary[p][q].Omega_)

                            # Sumx_KF = c*self.trackedAuxillary[p][q].q_ + Sumx_KF
                            # Sump_KF = c*(self.trackedAuxillary[p][q].Omega_) + Sump_KF
                            C = C + c

                            ##Only CI
                            # q_.append(self.trackedAuxillary[p][q].q_)
                            # Omega_.append(self.trackedAuxillary[p][q].Omega_)
                            # # print("q_",q_)
                            
                            # #only KF
                            Sumx_KF = self.trackedAuxillary[p][q].q_ + Sumx_KF
                            Sump_KF = (self.trackedAuxillary[p][q].Omega_) + Sump_KF
                    
                ## Only KF
                self.trackedObjects[p].q = self.trackedObjects[p].q + Sumx_KF
                self.trackedObjects[p].Omega = self.trackedObjects[p].Omega + Sump_KF
                self.trackedObjects[p].x = np.linalg.pinv(self.trackedObjects[p].Omega).dot(self.trackedObjects[p].q)
                self.trackedObjects[p].maint = 0
                            
                if q_ != []:

                        Sumx_CI,Sump_CI = self.CI(q_,Omega_)
                    
                        ## Only CI 
                        # self.trackedObjects[p].q = self.trackedObjects[p].q + Sumx_CI
                        # self.trackedObjects[p].Omega = self.trackedObjects[p].Omega + Sump_CI
                        # self.trackedObjects[p].x = np.linalg.pinv(self.trackedObjects[p].Omega).dot(self.trackedObjects[p].q)
                        # self.trackedObjects[p].maint = 0

                        # print( "\n Global Update Done! WIth CI")

                        # print("Omega Inverse is {} and x is {}".format(np.linalg.pinv(self.trackedObjects[p].Omega),self.trackedObjects[p].x))
                           
                self.counter.append(true_counter)
                # if (true_counter == 1 and c < 0.25 and f == 4):
                #     C = 0
                if true_counter>0:
                        averageHealth = C/true_counter

                else:
                        averageHealth = 0

                if (m != 0 and C!= 0):
                    
                        # SHARF
                        # self.trackedObjects[p].q = self.trackedObjects[p].q + Sumx_KF -(1-C/true_counter)*(Sumx_KF-Sumx_CI)
                        # self.trackedObjects[p].Omega = self.trackedObjects[p].Omega + Sump_KF - (1-C/true_counter)*(Sump_KF-Sump_CI)
                        # self.trackedObjects[p].x = np.linalg.pinv(self.trackedObjects[p].Omega).dot(self.trackedObjects[p].q)

                        # self.trackedObjects[p].maint = 0
                        # print( "\n Global Update Done! WIth KF")
                        # print("\n sum over q is {} and sum over Omega is {} and x is {} with m {}".format(Sumx,Sump,self.trackedObjects[p].x,m))
                        tracker = Tracker()
                        tracker.x = self.trackedObjects[p].x[0].item()
                        tracker.y = self.trackedObjects[p].x[1].item()
                        tracker.id = int(self.trackedObjects[p].ID)
                        curr_time = Clock().now()
                        tracker.header.stamp = curr_time.to_msg()
                        #self.Filter.tracker.append(tracker)
                        self.pubFilter.publish(tracker)
                        if sensor_id == 3:
                            self.sensor_x_error.append(data.sensor[i].x - self.trackedObjects[0].x[0])
                            self.sensor_y_error.append(data.sensor[i].y - self.trackedObjects[0].x[1])                        
                        #print("self.tracked x is {}".format(self.trackedObjects[0].x[0]))
                        self.track1[0].append(0)
                        self.track1[1].append(0)
                        self.track1[2].append(self.trackedObjects[0].x[0])
                        self.track1[3].append(self.trackedObjects[0].x[1])
                        # if self.track1:
                        #     self.ax.clear()
                        #     self.ax.scatter(self.sensor_x_imu, self.sensor_y_imu, color= self.color_arr[0], label='IMU', s=4)
                        #     self.ax.scatter(self.sensor_x_gps, self.sensor_y_gps,  color=self.color_arr[1],label='GPS Vectornav', s=4)
                        #     self.ax.scatter(self.sensor_x_gps_vehicle, self.sensor_y_gps_vehicle,  color=self.color_arr[2],label='GPS Low Res', s=4)
                        #     # self.ax.scatter(self.sensor_x_odom, self.sensor_y_odom,  color=self.color_arr[2],label='Odometry', s=4)
                        #     # self.ax.scatter(self.sensor_x, self.sensor_y, c='red', label='Sensor Data', s=3)
                        #     self.ax.plot(self.piksi_append[0],self.piksi_append[1],label='Piksi')
                        #     self.ax.plot(self.track1[2],self.track1[3], color=self.color_arr[4], label='Filter')
                        #     #self.ax.plot(piksi[0],piksi[1],label='Piksi')
                        #     self.ax.set_xlabel('X(m)')
                        #     self.ax.set_ylabel('Y(m)')
                        #     self.ax.legend()
                        # plt.pause(0.01)
                        if true_counter >0:
                            erx1= self.trackedObjects[p].x[0]- self.piksi[0]
                            ery1= self.trackedObjects[p].x[1]- self.piksi[1]
                            # self.file1.write(str(curr_time.nanoseconds/1e9) +"," + str(self.trackedObjects[p].x[0].item()) +"," + str(erx1.item()) + "," +str(ery1.item()) +"," +str(self.trackedObjects[p].x[2].item)+","+ str(averageHealth)+ "\n")
                            # self.file1.write(str(curr_time.nanoseconds/1e9) +"," + str(self.trackedObjects[p].x[0].item()) +"," + str(erx1.item()) + "," +str(ery1.item()) +"," +str(self.trackedObjects[p].x[2].item)+","+ str(averageHealth)+ ","+str(self.trackedHealth[1][0].x[0].item())+"\n")
                            Track_dict = {
                                            'Filter X': self.trackedObjects[0].x[0].item(),
                                            'Filter Y': self.trackedObjects[0].x[1].item(),
                                            'IMU X': self.imu_x[0],
                                            'IMU Y' : self.imu_y[0],
                                            'GPS X' : self.gps_x[0],
                                            'GPS Y' : self.gps_y[0],
                                            'Ground Truth X': self.piksi[0],
                                            'Ground Truth Y': self.piksi[1],
                                            'Algorithm': "KFNO",
                                            'Spoofing Rate': 2

                                         }
                            self.df.append(Track_dict)
                            # self.File1.append(np.array([curr_time,self.piksi[1],self.trackedObjects[p].x[0], self.trackedObjects[p].x[1],erx1,ery1,self.trackedObjects[p].x[2],averageHealth]))

                # if self.trackedHealth[1][0].start > 0:
                #        self.file2.write(str(self.piksi[1])+","+str(self.trackedHealth[1][0].x[0])+"\n")

 
	####################################################################################
	## Covariance Intersection                                                    
    def CI(self,q_,omega_):
        t = self.get_clock().now().nanoseconds/10e9
        n = np.shape(q_)[0]
        c = np.zeros((n),dtype = float)
        A_eq = np.ones((1,n),dtype = float)
        for i in range(n):    
            c[i] = np.trace(omega_[i])
        result = linprog(c.T, A_ub = None, b_ub = None, A_eq = A_eq, b_eq = 1, bounds = [0,1])
        w_ = result.x
        # w = cp.Variable(n)
        # prob = cp.Problem(cp.Minimize(c.T @ w),[A_eq @ w == 1])
        # prob.solve()
        # w_= w.value
        # print("In slover",w_)
        Sumx_CI = np.zeros((self.x_size,1), dtype = float)
        Sump_CI = np.zeros((self.x_size,self.x_size), dtype = float)
        W = 0.00
        for i in range(n):
            W = w_[i] + W
            Sumx_CI = w_[i]*q_[i] + Sumx_CI
            Sump_CI = w_[i]*omega_[i] + Sump_CI
        Sumx_CI = Sumx_CI/W
        Sump_CI = Sump_CI/W
        dt = self.get_clock().now().nanoseconds/10e9-t
        # print("Inside CI the weights are:{} {} and dt is{} \n ".format(w_,q_,dt))

        return Sumx_CI, Sump_CI

    def predictTrack(self, _dt ):
        dt = _dt
        if self.start == 0:
            self.trackedObjects.append(KalmanFilter())
            self.start = 1
        if self.start == 1:
            j=0
            self.numTrackedObjects = len(self.trackedObjects)
            # j = jth target, i = ith measurement
            for j in range(self.numTrackedObjects):
                self.trackedObjects[j].predict(dt)
                #print(self.trackedObjects[j].x)
                
    def get_mahalonobis(self, _data, _flag,flag_):
            data = []
            #if self.finished_cycle==True: #To handle data coming at the same time!
            self.finished_cycle = False
            self.numMeasurements = len(_data.x)
            self.numTrackedObjects = len(self.trackedObjects)
            for k in range(len(_data.x)):
                data_ = np.array([[_data.x[k]], [_data.y[k]]])
                data.append(data_)
            flag = _flag
            i = 0
            j = 0
            self.mahalanobisDistance = np.zeros((self.numTrackedObjects, self.numMeasurements))
            self.assignmentTable = np.zeros((self.numTrackedObjects, self.numMeasurements))
            for j in range(self.numTrackedObjects):
                if flag == 1:
                    self.Only_radar[j] += 1
                self.trackedObjects[j].Sinverse(flag)
                # Store inverse of innovation covariance, so that it is only computed once per object
                if np.linalg.det(self.trackedObjects[j].S) == 0:
                    print('Singularity')
                else:
                    Sinv = np.linalg.pinv(self.trackedObjects[j].S)
                    # Compute mahalanobis distance for each measurement i for the jth target
                    for i in range(self.numMeasurements):
                        #print('\nloop {},{}'.format(i,j))
                        self.trackedObjects[j].measurement(data[i], flag)
                        y = self.trackedObjects[j].y
                        #print('\tData {}: {}'.format(i, data[i]))
                        # print('\tResidual: {} and Sinv {}'.format(y,Sinv))
                        tmp = y.T.dot(Sinv).dot(y)
                        # print('tmp: {}'.format(tmp))
                        tmp = np.sqrt(tmp*np.sign(tmp))
                        self.mahalanobisDistance[j, i] = tmp
                        if (tmp < self.threshold):
                            self.assignmentTable[j, i] = 0
                        else:
                            self.assignmentTable[j, i] = 1
            self.optimal_cost_finder(data, flag,flag_)
            print("\n mahalonobis didtance: {}".format(self.mahalanobisDistance))
            data_= data
            
            
    def optimal_cost_finder(self, data, flag,flag_):
        self.assignment = False
        for i in range(np.shape(self.mahalanobisDistance)[1]):
            if (np.all(self.assignmentTable[:,i]) and len(self.trackedObjects)<1 and flag == 2 ):
                    print('No assignment!')
                    #print('\nNew Object ID: {}'.format(self.currID))
                    self.trackedObjects.append(KalmanFilter())
                    # Need to append new trackhealth and tracklets
                    self.trackedObjects[-1].set_state(data[i])
                    self.trackedObjects[-1].ID = self.currID
                    self.currID += 1
                    self.mahalanobisDistance = np.delete(self.mahalanobisDistance, i, axis=1)
                    #print('\n New track Created and Number of tracks is: {}'.format(len(self.trackedObjects)))
                    self.start = 1
                    self.assignment = True    
      
        if (self.mahalanobisDistance.size > 0 and self.assignment == False ):
            row_ind, col_ind = linear_sum_assignment(self.mahalanobisDistance)
            # print('\n Index {},{}: Mahal {}' .format(row_ind,col_ind,self.mahalanobisDistance))
            for  row, i in enumerate(col_ind):
                objNum = row_ind[row]
                if flag == 3: # if 10 else 2.5
                    threshold = 1000#5
                else:
                    threshold = 1000#5
                    #print('\n inside loop of measurement {} with objNum {} threshold is {}'.format(i,objNum,threshold))
                if (self.mahalanobisDistance[objNum][i]<threshold):
                    f = flag_ -1
                    # print("self.i",self.i,f,i)
                    t= Clock().now().nanoseconds
                    self.trackedHealth[f][objNum].predict(data[i],flag,t)
                    # Before accessing self.trackedAuxillary[objNum], check its existence
                    if objNum < len(self.trackedAuxillary) and len(self.trackedAuxillary[objNum]) > 0:
                        try:
                            self.trackedAuxillary[objNum][self.i].update(data[i], flag, flag_)
                            self.trackedAuxillary[objNum][self.i].Update_flag = True
                        except IndexError as e:
                            print(f"Error updating with objNum={objNum}, i={self.i}: {e}")
                    else:
                        print(f"Access error: objNum={objNum} is out of bounds or not initialized.")

                    # print('\n Track {} updated with measurement {} from flag {} (i={}) and x is {}'.format(objNum,data[i],flag,i,self.trackedAuxillary[objNum][self.i].x))
                    #print('\n Track health {} updated with flag {}  '.format(objNum,f))
                    if flag == 0:
                        self.Only_radar[objNum] = 0
                    if flag == 1:
                        self.Only_radar[objNum] += 1
        self.finished_cycle=True

def main():
    rclpy.init()
    node =  GNN_Filter_KF_NO()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

    
