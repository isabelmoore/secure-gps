#!/usr/bin/env python
import numpy as np
import math
class KalmanFilter:

    def __init__(self):
        # Each KF object gets a unique object ID
        self.ID = 0
        self.new = True
        self.flag_ = 5

        # Kalman Filter parameters
        self.x = np.array([[0], [0.], [0]], dtype=float)
        self.q = np.array([[0], [0.], [0]], dtype=float)
        self.P = np.diag([3., 2., 0.01])
        self.Omega = np.diag([0.3, 0.5, 100])
        self.S = np.zeros((2, 2), dtype=float)
        self.F = np.zeros((3, 3), dtype=float)
        self.y = np.zeros((2,1), dtype=float)
        self.K = np.zeros((3, 3), dtype=float)
        self.maint = 0
        self.H = np.array([[1, 0, 0],
                           [0, 1, 0]], dtype=float)

        # Sensor measurment noise matrix
        self.R_IMU = 0.01*np.diag([0.1,  0.1]) # this one!
        self.R_Odom = 0.1*np.diag([0.1, 0.1])
        self.R_GPS = 0.01*np.diag([0.1,0.1])

    ####################################################################
    # Computes the continous white noise model base on
    # Q = integrate(FQF^T)phi_x
    ####################################################################
    def continuous_white_noise(self, dt): # process noise for dynamic model, no integration
        spectral_density = 100
        Q = np.array([[dt**3 / 3,  dt**2 / 2,    0      ],
                      [0,          dt**3 / 3,  dt**2 / 2],
                      [0,           0,         dt**6 / 3]], dtype=float)*spectral_density
        return Q

    ####################################################################
    # Computes the piecewise white noise model base on
    # We should use 2D noise vector s.t. max(a_y) = 1/2*mu*g
    ####################################################################
    def piecewise_white_noise(self, dt): # process noise for dynamic model, takes integral of input matrix, 
        noise = 100
        Q = np.array([[dt**4 / 4, dt**3 / 2, dt**4 / 4],
                      [dt**3 / 2,   dt**2  , dt**3 / 2],
                      [dt**4 / 4, dt**3 / 2, dt**4 / 4]], dtype=float)*noise
        return Q

    def set_state(self, z):
        self.x[0] = z[0]
        self.x[1] = z[1]
        self.new = False

    ####################################################################
    # This funcion is the predict step of the linear Kalman Filter
    # xbar = Fx
    # Pbar = FPF^T + Q
    ####################################################################
    def predict(self, dt):
        self.Update_flag = False
        self.maint += 1
        self.Q = np.diag([0.2,0.2,0.1]) 
        self.Q = self.piecewise_white_noise(dt)
        self.W = np.linalg.pinv(self.Q)
        V = 4.5
        self.F = np.array([[1, 0,-dt*V*(math.sin(self.x[2]))],
                           [0, 1, dt*V*(math.cos(self.x[2]))],
                           [0, 0, 1                  ]], dtype=float)
        self.F_ = np.linalg.pinv(self.F)

        if self.maint < 10:

            self.x[0]  = self.x[0] +  dt*V*(math.cos(self.x[2]))
            self.x[1]  = self.x[1] + dt*V*(math.sin(self.x[2]))
            self.x[2] = self.x[2] 
            self.M = self.Omega + self.F_.T.dot(self.W).dot(self.F_)
            self.Omega = self.W - self.W.dot(self.F_).dot(np.linalg.pinv(self.M)).dot(self.F_.T.dot(self.W))
            
        else:
            self.x = self.x
            self.Omega = self.Omega
        self.H = np.array([[1., 0., 0.],[0., 1., 0.]], dtype=float)
        self.q = self.Omega.dot(self.x)

    def Sinverse(self,flag):
        self.H = np.array([[1., 0, 0],[0,  1, 0]], dtype=float)
        print(flag)
        if flag == 2:                  
            self.R= self.R_IMU 
        elif flag == 3:  
            self.R = self.R_GPS
        elif flag ==4:
            self.R = self.R_Odom
        self.S = self.R + self.H.dot(np.linalg.pinv(self.Omega)).dot(self.H.T)
            
    def measurement(self, z, flag):

        self.y = z - self.H.dot(self.x)

    
    ####################################################################
    # This function is the update step of the linear Kalman Filter
    ####################################################################
    def update(self, z, flag,flag_):
        self.maint = 0
        if flag == 2:                  
            self.R= self.R_IMU 
        elif flag == 3:  
            self.R = self.R_GPS
        elif flag ==4:
            self.R = self.R_Odom
               
        self.S = self.R+ self.H.dot(self.P).dot(self.H.T)
        self.y = z - self.H.dot(self.x)
        self.Omega = self.Omega + self.H.T.dot(np.linalg.pinv(self.R)).dot(self.H)
        self.q = self.q + self.H.T.dot(np.linalg.pinv(self.R)).dot(z)
            
        self.q_ = self.H.T.dot(np.linalg.pinv(self.R)).dot(z)
        self.Omega_ =  self.H.T.dot(np.linalg.pinv(self.R)).dot(self.H)

        self.flag_= int(flag_)
