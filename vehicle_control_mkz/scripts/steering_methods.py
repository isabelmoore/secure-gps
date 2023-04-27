#!/usr/bin/env python
import numpy as np
import pdb
from threading import Lock, Thread

wpmutex = Lock()

#### TO CHANGE ROS WORKING DIRECTORY
#### export ROS_HOME=$HOME 

#### USAGE
#    USE ESTIMATOR TO GET VEHICLE ODOM FIRST

class SteeringMethods:
    def __init__(self,filename='default',lookAhead=5.0,wheelBase=2.87,steeringRatio=14.8):
        self.flag = 0
        self.pathArray = []
        with open(filename,"r") as f: 
            for line in f:
                self.pathArray.append(line.strip())
        self.pathArray = np.array([list(map(float, x.split(','))) for x in self.pathArray])
        self.pathArray = np.array([[float(y) for y in x] for x in self.pathArray])

        self.LA = lookAhead
        self.WB = wheelBase
        self.Sr = steeringRatio	
        
    #### MISC FUNCTIONS
    def satValues(self,value,satLower, satUpper):
        if value >= satUpper:
            return satUpper
        elif value <= satLower:
            return satLower
        else:
            return value
    def readtext(self):
        self,filename = ('/odom_waypoints.dat ')
        txt  = np.loadtxt(filename,delimiter=',')
 
        return txt
    def angleDiff(self,a,b):
        diff = a-b 
        return np.fmod((diff+np.pi),2*np.pi)-np.pi
    
    def setParams(self,lookAhead,wheelBase,steeringRatio):
        self.LA = lookAhead
        self.WB = wheelBase
        self.Sr = steeringRatio	
        
    def setVehicleState(self,pose_x,pose_y,roll,pitch,yaw,linearX,angularZ):
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.roll = roll
        self.pitch = pitch 
        self.yaw = yaw 
        self.linearX = linearX 
        self.angularZ = angularZ
        self.flag = 1 
        
    def methodAdaptiveLookAhead(self,lMIn,lMax,gamma,poseX,poseY,yaw,linearX,curv):
            self.LA = linearX + 1 
            
    def methodAdaptiveVelocity(self,vMin,vMax,ayLim,curv):
        vCmd = 0.85* np.sqrt((ayLim/(abs(curv)+0.00005)))
        if vCmd >= vMax: 
            return vMax 
        elif vCmd <= vMin:
            return vMin
        else: 
            return vCmd
    def updateWayPoints(self,new_array):
        wpmutex.acquire()
        self.pathArray = new_array
        wpmutex.release()
    
    def methodPurePursuit(self,pose_x,pose_y,yaw):
        wpmutex.acquire()
        wheelBase = self.WB
        lookAhead = self.LA
        steeringRatio = self.Sr
        diff = self.pathArray - np.array([pose_x,pose_y]) #find closest waypoint
        diffSq = diff[:,0]**2 + diff[:,1]**2
        minInd = np.argmin(diffSq)
        distList = np.sqrt(diffSq)
        i = minInd
        while i>=0:
            if distList[i] >= lookAhead:
                targetPoint = i
                break
            else:
                i += 1
                if i > len(self.pathArray) - 1:
                    i = 0
        targetX = self.pathArray[targetPoint][0]
        targetY = self.pathArray[targetPoint][1]
        absoluteBearing = np.arctan2(targetY - pose_y, targetX - pose_x)
        #print("poses:",pose_y,pose_x,pose_y)
        relativeBearing = self.angleDiff(absoluteBearing,yaw)
        curv = 2*np.sin(relativeBearing)/distList[targetPoint]
        wpmutex.release()
        steercmd = steeringRatio*np.arctan2(wheelBase*curv,1)
        #print("method pursuit",targetPoint, minInd, distList[i], relativeBearing)
        return steercmd, curv, absoluteBearing,relativeBearing, targetPoint 
