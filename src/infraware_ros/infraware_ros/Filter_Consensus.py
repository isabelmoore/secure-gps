#!usr/bin/env python
import numpy as np
from copy import copy, deepcopy
import yaml
import math
from .KalmanFilter import KalmanFilter
from .HealthMonitor import HealthMonitor
import os
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from vision_msgs.msg import SensorH , SensorHCompact
from scipy.optimize import linear_sum_assignment, linprog
import matplotlib.pyplot as plt
#### To use KalmanFilter(backup) change the trackeobject[i].x[1] to trackeobject[i].x[2]
# from colorama import Fore, Style
trackNames = []
track = []
camera1 = []
camera2 = []
camera3 = []
camera4 = []
radar1 = []
track1 = []
Health11 = []
Health21 = []
Health31 = []
Health41 = []
Health51 = []
Health12 = []
Health22 = []
Health32 = []
Health42 = []
Health52 =[]
track2 = []
piksi = []
piksi2 = []
counter =[]



class GNN_Filter(Node):
    def __init__(self):
        super().__init__('Filter')
        # Variable to store all Kalman Filter objects
        self.finished_cycle=True
        self.start = 0
        self.Only_radar=[0,0]
        self.start = 0

        self.trackedObjects = []
        self.trackedHealth = np.empty((5,2), dtype = object)
        for i in range(5):
            for j in range(2):
                self.trackedHealth[i][j] = HealthMonitor()
        self.numTrackedObjects = 0
        self.mahalanobisDistance = np.array([])
        self.threshold = 8 #This will effect the new track creation
        self.numMeasurements = 0
        self.assignmentTable = np.array([])
        self.frameNumber = 0
        self.currID = 0
        # Sensor Class initialization

        self.piksi = []
        self.piksi2=[]
        self.piksiSmooth=0.000
        self.piksiSmooth2= 0.000

        # print(self.Radar)

        self.Filter = Odometry()
        self.Filter2 = Odometry()
        self.Filter3 = Odometry()

        self.pubFilter = self.create_publisher(Odometry,'/Filter', 10)
        self.pubFilter2 = self.create_publisher(Odometry,'/Filter2',  10)
        self.pubFilter3 = self.create_publisher(Odometry,'/Filter3',  10)


        self.create_subscription(PoseWithCovarianceStamped,'/piksi/enu_pose_fix',  self.piksi_callback,10)
        self.create_subscription(PoseWithCovarianceStamped,'/piksi2/enu_pose_fix', self.piksi2_callback,10)
        self.create_subscription(SensorHCompact,'/InfraSensorsData',  self.Infra_Sensor_callback,10)
        #rospy.Subscriber('/Piksi_Smooth', Vector3Stamped,self.PiksiSmooth_callback)
        #rospy.Subscriber('/Piksi2_Smooth', Vector3Stamped,self.PiksiSmooth2_callback)

        print('Filtering!')
        self.prev_time = Clock().now().nanoseconds




    def Infra_Sensor_callback(self,data):
        dt=0.05
        n = len(self.trackedObjects)
        m = len(data.sensor) 
        #print("input data is {}".format(data))
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
        self.i = 0
        #Without Radar:
        #self.start = 1
        if self.start ==1:
           #print("initial data",data)
           self.predictTrack(dt)
           

           
           self.trackedAuxillary =[]
           #print(n,m)
           self.trackedAuxillary = np.empty((n,m),dtype= object)
           for k in range(n):
               for j in range(m):
                   
                   
                   self.trackedAuxillary[k][j] = deepcopy(self.trackedObjects[k])
                   #print("after predict",self.trackedAuxillary[k][j].Update_flag)
                   #print("\n x after predict of track {} and measuremnt {} is : {}".format(k,j,self.trackedAuxillary[k][j].x))

           for i in range(m): #measurement
               if len(data.sensor[i].id) ==1:
                   
                   flag__ = np.array([data.sensor[i].id])
                   flag_ = np.asscalar(flag__[0])
               else:
                   flag__ = np.array([data.sensor[i].id[0]])
                   flag_ = np.asscalar(flag__[0])
               if flag_ == 5:
                    flag =1 #radar
               else:
                    flag =0 #camera
               #print("the flag is :{}".format(flag_))
               self.i = i
               self.get_mahalonobis(data.sensor[i],flag,flag_)

           ### Error here: for each track it use all measuremnts to update instead of the assigned one.
           
           for p in range(n):#tracked objects 
               Sumx_KF = np.zeros((4,1), dtype = float)
               Sump_KF = np.zeros((4,4), dtype = float)
               Sumx_CI = np.zeros((4,1), dtype = float)
               Sump_CI = np.zeros((4,4), dtype = float)
               C = 0.00
               true_counter = 0
               #print("In the first loop")
               q_= []
               Omega_ = []
               for q in range(m): #measurement 
                    
                    #print("\n Updated is {} for track {} and measurement {} ".format(self.trackedAuxillary[p][q].Update_flag,p,q))
                    if self.trackedAuxillary[p][q].Update_flag == True:
                      
                      true_counter += 1
                      f = self.trackedAuxillary[p][q].flag_ - 1
                      c = self.trackedHealth[f][p].x[0]
                      #q_.append(c*self.trackedAuxillary[p][q].q_)
                      #Omega_.append(c*self.trackedAuxillary[p][q].Omega_)

                      #Only CI
                      q_.append(self.trackedAuxillary[p][q].q_)
                      Omega_.append(self.trackedAuxillary[p][q].Omega_)
                      ##
                      C = C + c
                      Sumx_KF = c*self.trackedAuxillary[p][q].q_ + Sumx_KF
                      #print("\n health coeficent for measurment {} and track {}  is {} and flag is {}".format(q,p,self.trackedHealth[f][p].x,f))
                      #print("\n x ={}  for auxi track {} and measurement {}".format(self.trackedAuxillary[p][q].x,p,q))
                      #Sump = c*np.linalg.pinv(self.trackedAuxillary[p][q].P) + Sump
                      Sump_KF = c*(self.trackedAuxillary[p][q].Omega_) + Sump_KF
                      
               counter.append(true_counter)

               if (true_counter == 1 and c < 0.25 and f == 4):
                    C = 0
               #print("C",C)
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
                   #self.trackedObjects[p].x = Sumx/C
                   #self.trackedObjects[p].P = np.linalg.pinv(Sump/C)
                   self.trackedObjects[p].q = self.trackedObjects[p].q + Sumx_KF -(1-C/true_counter)*(Sumx_KF-Sumx_CI)
                   self.trackedObjects[p].Omega = self.trackedObjects[p].Omega + Sump_KF - (1-C/true_counter)*(Sump_KF-Sump_CI)
                   #self.trackedObjects[p].P = np.linalg.pinv(self.trackedObjects[p].Omega)
                   self.trackedObjects[p].x = np.linalg.pinv(self.trackedObjects[p].Omega).dot(self.trackedObjects[p].q)

                   self.trackedObjects[p].maint = 0
                   print( "\n Global Update Done!")
                   #print("\n sum over q is {} and sum over Omega is {} and x is {} with m {}".format(Sumx,Sump,self.trackedObjects[p].x,m))
               
           Health11.append(np.array([self.piksi.y,self.trackedHealth[0][0].x[0]]))
           Health21.append(np.array([self.piksi.y,self.trackedHealth[1][0].x[0]]))
           Health31.append(np.array([self.piksi.y,self.trackedHealth[2][0].x[0]]))
           Health41.append(np.array([self.piksi.y,self.trackedHealth[3][0].x[0]]))
           Health51.append(np.array([self.piksi.y,self.trackedHealth[4][0].x[0]]))
           Health12.append(np.array([self.piksi2.y ,self.trackedHealth[0][1].x[0]]))
           Health22.append(np.array([self.piksi2.y ,self.trackedHealth[1][1].x[0]]))
           Health32.append(np.array([self.piksi2.y ,self.trackedHealth[2][1].x[0]]))
           Health42.append(np.array([self.piksi2.y ,self.trackedHealth[3][1].x[0]]))
           Health52.append(np.array([self.piksi2.y ,self.trackedHealth[4][1].x[0]]))      
               #else:
                    #self.trackedObjects[p].x = Sumx
                    #self.trackedObjects[p].P = Sump




	####################################################################################
	## Covariance Intersection                                                    ######
	####################################################################################
    def CI(self,q_,omega_):
            n = np.shape(q_)[0]
        
            c = np.zeros((n),dtype = float)
            A_eq = np.ones((1,n),dtype = float)
            #print(n,c)
            for i in range(n):
                
                c[i]= np.trace(omega_[i])
                

            #print(n,A_eq,c.T)
            res = linprog(c.T, A_ub=None, b_ub=None, A_eq=A_eq, b_eq=1, bounds=[0,1])
            w_ = res.x
            print("HI There",n, w_)
            Sumx_CI = np.zeros((4,1), dtype = float)
            Sump_CI = np.zeros((4,4), dtype = float)
            W = 0.00
            for i in range(n):
                print("HI There",n, w_[i],q_[i],omega_[i])
                W = w_[i]+W
                Sumx_CI = w_[i]*q_[i]+ Sumx_CI
                Sump_CI = w_[i]*omega_[i]+ Sump_CI
            Sumx_CI = Sumx_CI/W
            Sump_CI = Sump_CI/W

            return Sumx_CI, Sump_CI
        

	####################################################################################
	## ROS subscriber for Piksi                                                   ######
	####################################################################################
        
    def piksi_callback(self, data):

        self.piksi = data.pose.pose.position
        piksi.append(np.array([self.piksi.x, self.piksi.y]))
    '''
    def PiksiSmooth_callback(self,data):
   
        self.piksiSmooth = data.vector
        self.piksiSmooth.x = self.piksi.x    
    '''
    def piksi2_callback(self, data):
       
        self.piksi2 = data.pose.pose.position
        piksi2.append(np.array([self.piksi2.x, self.piksi2.y]))
    '''
    def PiksiSmooth2_callback(self,data):
   
        self.piksiSmooth2 = data.vector
        self.piksiSmooth2.x = self.piksi2.x   
    '''

    def predictTrack(self, _dt ):
        dt = _dt
        if self.start == 0:
            self.trackedObjects.append(KalmanFilter())
            self.start =1
        if self.start ==1:
            j=0
            
            #print('\nonly radar is used to update {}' .format(self.Only_radar))
            #print('\nData currently being processed {} :{}'.format(data,camflag))
           
            self.numTrackedObjects = len(self.trackedObjects)

            #print(self.numTrackedObjects)
            #print('\tno.Tracks: {} '.format(self.numTrackedObjects))


            ####################################################################
            # j = jth target, i = ith measurement
            ####################################################################
            lr = [1.5185,2.3]
            lf = [1.3314, 1.5]
            q=[10,10]

            for j in range(self.numTrackedObjects):
            
                self.trackedObjects[j].predict(dt,self.Only_radar)
                #print(self.trackedObjects[j].x)
                

        


    def get_mahalonobis(self, _data, _flag,flag_):
            data =[]
        
            #if self.finished_cycle==True: #To handle data coming at the same time!
            self.finished_cycle = False
            self.numMeasurements = len(_data.x)
            self.numTrackedObjects = len(self.trackedObjects)
            for k in range(len(_data.x)):
                data_ = np.array([[_data.x[k]], [_data.y[k]]])
                data.append(data_)
            #print("Inside get mahal")
            flag = _flag
            i=0
            j=0
            #print("Hi")
            self.mahalanobisDistance = np.zeros((self.numTrackedObjects, self.numMeasurements))
            self.assignmentTable = np.zeros((self.numTrackedObjects, self.numMeasurements))
            for j in range(self.numTrackedObjects):
                if flag ==1:
                    self.Only_radar[j] += 1


            
            
                # if self.trackedObjects[j].new == True:
                #     self.trackedObjects[j].x = np.array([])
                #print('ID: {}'.format(self.trackedObjects[j].ID))
                
                
            
                self.trackedObjects[j].Sinverse(flag)

                # Store inverse of innovation covariance, so that it is only computed once per object
                if np.linalg.det(self.trackedObjects[j].S) == 0:
                    print('Singularity')
                else:
                    Sinv = np.linalg.pinv(self.trackedObjects[j].S)
                    #print("S", self.trackedObjects[j].S)
                    # Compute mahalanobis distance for each measurement i for the jth target

                    for i in range(self.numMeasurements):
                    
                    
                        #print('\nloop {},{}'.format(i,j))
                        self.trackedObjects[j].measurement(data[i], flag)
                        y = self.trackedObjects[j].y
                        #print('\tData {}: {}'.format(i, data[i]))
                        #print('\tResidual: {}'.format(y))
                        tmp = y.T.dot(Sinv).dot(y)
                        #print('tmp: {}'.format(tmp))
                        tmp = np.sqrt(tmp)
                        self.mahalanobisDistance[j, i] = tmp
                        if (tmp < self.threshold):
                            self.assignmentTable[j, i] = 0
                        else:
                            self.assignmentTable[j, i] = 1
        
        
            

            # print('Num Measurements: ', self.numMeasurements)
            # if (self.numTrackedObjects == 0):
            #     self.optimal_cost_finder(data, flag)
            #print("mahalobnobis:",self.mahalanobisDistance)
            self.optimal_cost_finder(data, flag,flag_)
            #print(self.finished_cycle)
            data_= data
            
            

    def optimal_cost_finder(self, data, flag,flag_):
        #print(self.mahalanobisDistance)
        #print('Updating Tracked Objects!')
        #print('\tNum Tracked Obj: {}'.format(self.numTrackedObjects))
        #print('\tMahalanobis Distance: {}'.format(self.mahalanobisDistance))
        #print('\nAssignment Table: {}'.format(self.assignmentTable))
        y0=np.zeros(2,dtype=float)
        y1=np.zeros(2,dtype=float)
        P0=np.zeros(2,dtype=float)
        P2=np.zeros(2,dtype=float)

        (n,m) = self.mahalanobisDistance.shape
        #n : Number ofObjects m: Number of measurements
        #max_cost = np.zeros((n,2))
 
        #n = [[5], [5]]

        # maxValues = np.argmax(obj, axis=1)
        self.assignment = False

        for i in range(m):
            #print("HI")
            if (np.all(self.assignmentTable[:,i]) and flag ==0 and len(self.trackedObjects)<2):

                    #print('No assignment!')
                
                    #print('\nNew Object ID: {}'.format(self.currID))
                    self.trackedObjects.append(KalmanFilter())
                    self.trackedObjects[-1].set_state(data[i], flag)
                    self.trackedObjects[-1].ID = self.currID
                    track.append(np.array([self.trackedObjects[-1].x[0], self.trackedObjects[-1].x[2]]))
                    trackNames.append('track' + str(self.currID) + '.txt')
                    self.currID += 1
                    self.mahalanobisDistance = np.delete(self.mahalanobisDistance, i, axis=1)
                    #del data[i]
                    #print('\n New track Created and Number of tracks is: {}'.format(len(self.trackedObjects)))
                    self.start =1
                    self.assignment = True
            
      
        #the filter cannot handle no assignments       
        if (self.mahalanobisDistance.size > 0 and self.assignment==False ):


                    row_ind, col_ind = linear_sum_assignment(self.mahalanobisDistance)
                    #print('\n Index {},{}: Mahal {}' .format(row_ind,col_ind,self.mahalanobisDistance))
                    for  row, i in enumerate(col_ind):
                            objNum = row_ind[row]
                            if flag == 1: # if 10 else 2.5
                                threshold = 3#5
                            else:
                                threshold = 3#5
                            #print('\ninside loop of measurement {} with objNum {} threshold is {}'.format(i,objNum,threshold))
                            if (self.mahalanobisDistance[objNum][i]<threshold):
                                f = flag_ -1
                                #print("self.i",self.i,f)
                                t= Clock().now().nanoseconds
                                self.trackedHealth[f][objNum].predict(data[i],flag,t)
                                self.trackedAuxillary[objNum][self.i].update(data[i], flag, flag_)
                                self.trackedAuxillary[objNum][self.i].Update_flag = True
                                #print("hello", self.trackedAuxillary[objNum][self.i].x)
                                #y0[objNum]= self.trackedObjects[objNum].y[0]
                                #y1[objNum]= self.trackedObjects[objNum].y[1]
                                #P0[objNum]= self.trackedObjects[objNum].P[0][0]
                                #P2[objNum]= self.trackedObjects[objNum].P[2][2]
                                
                                #print('\n Track {} updated with measurement {} (i={}) and x is {}'.format(objNum,self.i,i,self.trackedAuxillary[objNum][self.i].x))
                                #print('\n Track health {} updated with flag {}  '.format(objNum,f))
                                if flag == 0:
                                    self.Only_radar[objNum]=0
                                if flag ==1:
                                    self.Only_radar[objNum] += 1
                                    #print('\nonly radar is used to update {}' .format(self.Only_radar))
                    '''
                    if (self.numMeasurements >= self.numTrackedObjects): 
                        maxIndex = np.argmin(self.mahalanobisDistance, axis=1)
                        print('\nIndex {}: Mahal {}' .format(maxIndex,self.mahalanobisDistance))
                        for  objNum, i in enumerate(maxIndex):
                            
                            if self.Only_radar[objNum]>5:
                               threshold = 10
                            else:
                                threshold = 2.5 # Change to 2.5 for two car
                           

                            print('\ninside loop of mmeasurement {} with objNum {}'.format(i,objNum))
                            if (self.mahalanobisDistance[objNum][i]<threshold):
                                self.trackedObjects[objNum].update(data[i], flag)
                                print('Track {} updated!'.format(objNum))
                                if flag == 0:
                                    self.Only_radar[objNum]=0

                            
                                
                    else:
                        maxIndex = np.argmin(self.mahalanobisDistance, axis=0)
                        print('\nIndex {}: Mahal {}' .format(maxIndex,self.mahalanobisDistance))
                        for objNum in maxIndex:
                            i=0
                            print("We are here")
                            if self.Only_radar[objNum]>5:
                               threshold = 10
                            else:
                                threshold =5 # Change to 2.5 for two car
                            if (self.mahalanobisDistance[objNum][i]<threshold):
                                self.trackedObjects[objNum].update(data[i], flag)
                                if flag == 0:
                                    self.Only_radar[objNum]=0

                                

                    
                    

                            #print('x: ', self.trackedObjects[objNum].x)
                            #print("HELLO")
                        #else:
                            #print('Object via no data!')
                            # print('x: ', self.trackedObjects[j].x)
                            #self.trackedObjects[objNum].set_state(data[i], flag)
                            # print('x: ', self.trackedObjects[j].x)
                    #for i, obj in enumerate(self.trackedObjects):
                    '''
                    erx1= self.trackedObjects[0].x[0]- self.piksi.x
                    ery1= self.trackedObjects[0].x[2]- self.piksi.y
                    track1.append(np.array([self.piksi.y,self.trackedObjects[0].x[0], self.trackedObjects[0].x[2],erx1,ery1,self.trackedObjects[0].x[3]]))
                    self.Filter.pose.pose.position.x = self.trackedObjects[0].x[0]
                    self.Filter.pose.pose.position.y = self.trackedObjects[0].x[2]
                    self.Filter.header.frame_id = str(self.trackedObjects[0].ID)
                    curr_time = Clock().now()
                    self.Filter.header.stamp = curr_time.to_msg()
                    self.pubFilter.publish(self.Filter)
                
                    if len(self.trackedObjects)>1:
                       erx2= self.trackedObjects[1].x[0]- self.piksi2.x
                       ery2= self.trackedObjects[1].x[2]- self.piksi2.y
                       track2.append(np.array([self.piksi2.y,self.trackedObjects[1].x[0], self.trackedObjects[1].x[2],erx2,ery2,self.trackedObjects[1].x[3]]))    
                       self.Filter2.pose.pose.position.x = self.trackedObjects[1].x[0]
                       self.Filter2.pose.pose.position.y = self.trackedObjects[1].x[2]
                       self.Filter2.header.frame_id = str(self.trackedObjects[1].ID)
                       curr_time = Clock().now()
                       self.Filter2.header.stamp = curr_time.to_msg()
                       self.pubFilter2.publish(self.Filter2)
                    



        #print('Optimal Found!')
        
        self.finished_cycle=True

def shutdown():
    # fileNames = ['piksi.txt', 'track0.txt', 'track1.txt', 'track2.txt', 'track3.txt']
    # data = [piksi, track0, track1, track2, track3]
    camNames = ['track1.txt','track2.txt','piksi1.txt','piksi2.txt','Health11.txt','Health21.txt','Health31.txt','Health41.txt','Health51.txt','Health12.txt','Health22.txt','Health32.txt','Health42.txt','Health52.txt','counter']
    cams = [track1, track2, piksi, piksi2,Health11,Health21,Health31,Health41,Health51,Health12,Health22,Health32,Health42,Health52,counter ]
    

    for i in range(len(cams)):
        np.savetxt(camNames[i], cams[i], delimiter=',')

def main():
    rclpy.init()
    #rclpy.on_shutdown(shutdown)
    node =  GNN_Filter()
    rclpy.spin(node)
    shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    