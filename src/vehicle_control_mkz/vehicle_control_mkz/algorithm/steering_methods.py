#!/usr/bin/env python
import numpy as np
import pdb
from threading import Lock, Thread

from vehicle_control_mkz.utility.shared_functions import sat_values


class SteeringMethods:
    def __init__(self, look_ahead_distance=5.0, wheelbase=2.87, steering_ratio=14.8, path_file=None,):
        # Read in parameters
        self.look_ahead_distance = look_ahead_distance
        self.wheelbase = wheelbase
        self.steering_ratio = steering_ratio	
        
        # Initialize flags
        self.got_state = False
        self.got_path = False
        
        # If a path file was given, read it in
        if path_file is not None:
            self.read_path(path_file)
        
        # Create a mutex lock to ensure that the path is not updated while it is being used
        self.mutex_lock = Lock()

    # Reads in path from file into array
    def read_path(self, filename):
        # Re-initialize path array
        self.path_array = []
        
        # Read in path file
        with open(filename,"r") as f:
            for line in f:
                self.path_array.append(line.strip())
        self.path_array = np.array([list(map(float, x.split(','))) for x in self.path_array])
        self.path_array = np.array([[float(y) for y in x] for x in self.path_array])
        
        self.got_path = True

    # Set path array directly
    def set_path(self, new_path_array):
        # Lock the path array while it is being updated
        self.mutex_lock
        
        # Update the path array
        self.path_array = new_path_array
        
        # Unlock the path array
        self.mutex_lock.release()
        self.got_path = True


    # Take in two radians and return the minimum angle difference between them
    def angle_diff(self, a, b):
        diff = a-b 
        return np.fmod((diff+np.pi),2*np.pi)-np.pi


    # Update parameters
    def set_params(self, lookAhead, wheelBase, steeringRatio):
        self.look_ahead_distance = lookAhead
        self.wheelbase = wheelBase
        self.steering_ratio = steeringRatio	


    # Update vehicle state
    def set_vehicle_state(self, pose_x, pose_y, roll, pitch, yaw, linearX, angularZ):
        self.pose_x = pose_x
        self.pose_y = pose_y
        self.roll = roll
        self.pitch = pitch 
        self.yaw = yaw 
        self.linearX = linearX 
        self.angularZ = angularZ

        self.got_state = True

    # Calculate lookahead distance based on velocity
    def update_lookahead(self, linearX):
        self.look_ahead_distance = linearX + 1 
    
    # Calculate reduced velocity based on curvature
    def method_adaptive_velocity(self, vMin, vMax, ayLim, curv):
        vCmd = 0.85* np.sqrt((ayLim/(abs(curv)+0.00005)))
        if vCmd >= vMax: 
            return vMax 
        elif vCmd <= vMin:
            return vMin
        else: 
            return vCmd


    def method_pure_pursuit(self,pose_x,pose_y,yaw):
        # Lock the path array while it is being used
        self.mutex_lock.acquire()
        
        # Find the closest point on the path to the vehicle
        diff = self.path_array - np.array([pose_x,pose_y])
        diffSq = diff[:,0]**2 + diff[:,1]**2
        minInd = np.argmin(diffSq)
        distList = np.sqrt(diffSq)
        
        i = minInd
        while i>=0:
            if distList[i] >= self.look_ahead_distance:
                targetPoint = i
                break
            else:
                i += 1
                if i > len(self.path_array) - 1:
                    i = 0
        targetX = self.path_array[targetPoint][0]
        targetY = self.path_array[targetPoint][1]

        absoluteBearing = np.arctan2(targetY - pose_y, targetX - pose_x)
        relativeBearing = self.angle_diff(absoluteBearing,yaw)
        curv = 2*np.sin(relativeBearing)/distList[targetPoint]
        
        # Release the path array
        self.mutex_lock.release()
        steercmd = self.steering_ratio * np.arctan2(self.wheelbase * curv, 1)
        
        return steercmd, curv, absoluteBearing,relativeBearing, targetPoint 


    def get_path(self):
        return self.path_array
