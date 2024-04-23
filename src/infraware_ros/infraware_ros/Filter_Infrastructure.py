#!usr/bin/env python
import numpy as np
from copy import deepcopy
from scipy.optimize import linear_sum_assignment, linprog
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from vision_msgs.msg import SensorH , SensorHCompact, Tracker, TrackerArray
from swiftnav_ros2_driver.msg import Baseline
import matplotlib.pyplot as plt
from .KalmanFilter import KalmanFilter
from .HealthMonitor import HealthMonitor

class GNN_Filter(Node):
    def __init__(self):
        super().__init__('FilterInfra')

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
        self.threshold = 12 #This will effect the new track creation
        self.numMeasurements = 0
        self.assignmentTable = np.array([])
        self.frameNumber = 0
        self.piksiSmooth=0.000
        self.piksiSmooth2= 0.000
        self.track1 = [[],[],[],[]]
        self.Filter = TrackerArray()
        self.trackedHealth = np.empty((5,2), dtype = object)
        for i in range(5):
            for j in range(2):
                self.trackedHealth[i][j] = HealthMonitor()
        
        #publish filtered tracks
        self.pubFilter = self.create_publisher(TrackerArray,'infraware/filter_infra', 10)

        #subscribe to gps and sensor data in cartesian coordiantes
        self.create_subscription(Baseline, '/baseline',  self.piksi_callback,10)
        self.create_subscription(SensorHCompact,'/InfraSensorsData',  self.Sensor_callback,10)

        print('Filtering!')
        self.prev_time = Clock().now().nanoseconds

        #Initials plots
        self.fig,self.ax = plt.subplots()
        self.ax.set_xlabel('X(m)')
        self.ax.set_ylabel('Y(m)')

	#######################################################
	## ROS subscriber for Piksi                                                        
    def piksi_callback(self, data):
        self.piksi = [data.baseline_n_m,data.baseline_e_m,data.baseline_d_m]
    #######################################################
    ### Sensor Subscriber
    def Sensor_callback(self,data):
        dt = 0.05
        n = len(self.trackedObjects)
        m = len(data.sensor) 
        print("\t No of Track is {} and No of measurement is {} \n".format(n,m))
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
                   #print("\n x after predict of track {} and measuremnt {} is : {}".format(k,j,self.trackedAuxillary[k][j].x))
           
           for i in range(m): #measurement
                if data.sensor[i].id:
                    #print("data length is {} and i is {}".format(len(data.sensor),i))
                    if len(data.sensor[i].id) == 1:
                        flag__ = np.array([data.sensor[i].id])
                        flag_ = np.asscalar(flag__[0])
                    else:
                        flag__ = np.array([data.sensor[i].id[0]])
                        flag_ = np.asscalar(flag__[0])
                    if flag_ == 5:
                        flag = 1 #radar
                    else:
                        flag = 0 #camera
                    #print("the flag is :{}".format(flag_))
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
                    if data.sensor[i].id: 
                        print("\n Updated is {} for track {} and measurement {} ".format(self.trackedAuxillary[p][q].Update_flag,p,q))
                        if self.trackedAuxillary[p][q].Update_flag == True:
                    
                            true_counter += 1
                            f = self.trackedAuxillary[p][q].flag_ - 1
                            c = self.trackedHealth[f][p].x[0]
                            q_.append(c*self.trackedAuxillary[p][q].q_)
                            Omega_.append(c*self.trackedAuxillary[p][q].Omega_)

                            #Only CI
                            #q_.append(self.trackedAuxillary[p][q].q_)
                            #Omega_.append(self.trackedAuxillary[p][q].Omega_)
                            C = C + c
                            Sumx_KF = c*self.trackedAuxillary[p][q].q_ + Sumx_KF
                            #print("\n health coeficent for measurment {} and track {}  is {} and flag is {}".format(q,p,self.trackedHealth[f][p].x,f))
                            #print("\n x ={}  for auxi track {} and measurement {}".format(self.trackedAuxillary[p][q].x,p,q))
                            #Sump = c*np.linalg.pinv(self.trackedAuxillary[p][q].P) + Sump
                            Sump_KF = c*(self.trackedAuxillary[p][q].Omega_) + Sump_KF
                      
                self.counter.append(true_counter)
                if (true_counter == 1 and c < 0.25 and f == 4):
                    C = 0
                if q_ != []:
                    Sumx_CI,Sump_CI = self.CI(q_,Omega_)
                    '''
                    ## Only CI 
                    self.trackedObjects[p].q = self.trackedObjects[p].q + Sumx_CI
                    self.trackedObjects[p].Omega = self.trackedObjects[p].Omega + Sump_CI
                    #self.trackedObjects[p].P = np.linalg.pinv(self.trackedObjects[p].Omega)
                    self.trackedObjects[p].x = np.linalg.pinv(self.trackedObjects[p].Omega).dot(self.trackedObjects[p].q)

                    self.trackedObjects[p].maint = 0
                    ## 
                    '''
               
                if (m != 0 and C!= 0):
                    if data.sensor[i].id:
                        self.trackedObjects[p].q = self.trackedObjects[p].q + Sumx_KF -(1-C/true_counter)*(Sumx_KF-Sumx_CI)
                        self.trackedObjects[p].Omega = self.trackedObjects[p].Omega + Sump_KF - (1-C/true_counter)*(Sump_KF-Sump_CI)
                        self.trackedObjects[p].x = np.linalg.pinv(self.trackedObjects[p].Omega).dot(self.trackedObjects[p].q)

                        self.trackedObjects[p].maint = 0
                        print( "\n Global Update Done!")
                        #print("\n sum over q is {} and sum over Omega is {} and x is {} with m {}".format(Sumx,Sump,self.trackedObjects[p].x,m))
                        tracker = Tracker()
                        tracker.x = self.trackedObjects[p].x[0].item()
                        tracker.y = self.trackedObjects[p].x[1].item()
                        tracker.id = int(self.trackedObjects[p].ID)
                        curr_time = Clock().now()
                        tracker.header.stamp = curr_time.to_msg()
                        self.Filter.tracker.append(tracker)
                        self.pubFilter.publish(self.Filter)
                    
                        #print("self.tracked x is {}".format(self.trackedObjects[0].x[0]))
                        self.track1[0].append(0)
                        self.track1[1].append(0)
                        self.track1[2].append(self.trackedObjects[0].x[0])
                        self.track1[3].append(self.trackedObjects[0].x[1])
                        if self.track1:
                            self.ax.clear()
                            #self.ax.plot(piksi[0],piksi[1],label='Piksi')
                            self.ax.plot(self.track1[2],self.track1[3],label='Filter')
                            self.ax.set_xlabel('X(m)')
                            self.ax.set_ylabel('Y(m)')
                        plt.pause(0.01) 
 
	####################################################################################
	## Covariance Intersection                                                    
    def CI(self,q_,omega_):
        n = np.shape(q_)[0]
        c = np.zeros((n),dtype = float)
        A_eq = np.ones((1,n),dtype = float)
        for i in range(n):    
            c[i] = np.trace(omega_[i])
        result = linprog(c.T, A_ub = None, b_ub = None, A_eq = A_eq, b_eq = 1, bounds = [0,1])
        w_ = result.x
        Sumx_CI = np.zeros((self.x_size,1), dtype = float)
        Sump_CI = np.zeros((self.x_size,self.x_size), dtype = float)
        W = 0.00
        for i in range(n):
            W = w_[i] + W
            Sumx_CI = w_[i]*q_[i] + Sumx_CI
            Sump_CI = w_[i]*omega_[i] + Sump_CI
        Sumx_CI = Sumx_CI/W
        Sump_CI = Sump_CI/W

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
                        print('\tResidual: {} and Sinv {}'.format(y,Sinv))
                        tmp = y.T.dot(Sinv).dot(y)
                        print('tmp: {}'.format(tmp))
                        tmp = np.sqrt(tmp*np.sign(tmp))
                        self.mahalanobisDistance[j, i] = tmp
                        if (tmp < self.threshold):
                            self.assignmentTable[j, i] = 0
                        else:
                            self.assignmentTable[j, i] = 1
            self.optimal_cost_finder(data, flag,flag_)
            data_= data
            
            
    def optimal_cost_finder(self, data, flag,flag_):
        self.assignment = False
        for i in range(np.shape(self.mahalanobisDistance)[1] and len(self.trackedObjects)<2):
            if (np.all(self.assignmentTable[:,i]) and flag == 0):
                    #print('No assignment!')
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
            print('\n Index {},{}: Mahal {}' .format(row_ind,col_ind,self.mahalanobisDistance))
            for  row, i in enumerate(col_ind):
                objNum = row_ind[row]
                if flag == 1: # if 10 else 2.5
                    threshold = 10#5
                else:
                    threshold = 10#5
                    #print('\n inside loop of measurement {} with objNum {} threshold is {}'.format(i,objNum,threshold))
                if (self.mahalanobisDistance[objNum][i]<threshold):
                    f = flag_ -1
                    print("self.i",self.i,f,i)
                    t= Clock().now().nanoseconds
                    self.trackedHealth[f][objNum].predict(data[i],flag)
                    self.trackedAuxillary[objNum][self.i].update(data[i], flag, flag_)
                    self.trackedAuxillary[objNum][self.i].Update_flag = True
                    #print('\n Track {} updated with measurement {} (i={}) and x is {}'.format(objNum,self.i,i,self.trackedAuxillary[objNum][self.i].x))
                    #print('\n Track health {} updated with flag {}  '.format(objNum,f))
                    if flag == 0:
                        self.Only_radar[objNum] = 0
                    if flag == 1:
                        self.Only_radar[objNum] += 1
        self.finished_cycle=True

def main():
    rclpy.init()
    node =  GNN_Filter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
