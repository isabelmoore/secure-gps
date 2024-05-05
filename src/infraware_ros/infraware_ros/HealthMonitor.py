#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
# import pandas as pd

class HealthMonitor:

    def __init__(self, logger):
        self.start = 0
        self.new = True
        self.t_now = 0
        self.x = np.array([1, 0], dtype=float)
        self.H = 1
        self.H_alg = 1
        self.H_dym = 1
        self.a =0
        self.dy=0
        self.logger = logger
        self.health_data = []
        self.time_steps = []
        self.dm_data = []
        self.dmdt_data = []
        self.start = 0
        # self.plot_timer = self.create_timer(30, self.plot_all_health_data)  # Every 30 seconds

    def errfunction(self,x):
        erf = 0.5*(1+math.erf(3*x))
        erf = 0.5*(1+math.erf(6*x-3))
        return erf


    ''' Health Monitoring'''

    def predict(self,z ,flag, t): 
        if self.start <3 :
            self.z__ = z
            self.z_= z
            self.z = z
            self.start += 1
        else:
            self.z__ = self.z_
            self.z_ = self.z
            self.z = z
        self.L = np.array([[0.9989,  0.0011],
                           [0.0011,  0.9989]], dtype=float)
        
        ### Defining Health Measurement Function ####
        if (flag == 3): 
            ax = self.z[0]-2*self.z_[0]+ self.z__[0]
            ay = self.z[1]-2*self.z_[1]+ self.z__[1]
            self.a = np.sqrt(ax**2 + ay**2)
            self.H_1 = 1- 3*np.sign(self.a)* self.a
            if self.H_1 < 0 :
                self.H_1 = 0
        
            self.dy = self.z[0]-self.z_[0]
            self.H_2 = 1-np.sign(self.dy)*self.dy/1
            if self.H_2 < 0:
                self.H_2 = 0
            self.H= min(self.H_1,self.H_2)
        else:
            self.H = 1
        # Assuming some predefined thresholds based on system requirements
        # dt = t - self.t_now
        # self.t_now = t

        # # Calculate delta m based on the Algebraic Health Assessment formula
        # dm = np.sqrt((self.z[0] - self.z_[0])**2 + (self.z[1] - self.z_[1])**2)
        # # self.logger.info(f"Delta m: {dm}")  
        # self.dm_data.append(dm)      
        # alg_low = 0.0
        # alg_high = 0.2

        # # # Using the Algebraic Health Assessment
        # if dm < alg_low:
        #     self.H_alg = 1
        # elif dm > alg_high:
        #     self.H_alg = 0
        # else:
        #     # A quadratic function of Δm between the two thresholds
        #     self.H_alg = 1 - ((dm - alg_low)**2 / (alg_high - alg_low)**2)

        # dyn_low = 0.0
        # dyn_high = 1*(1**-7)
        # # Calculate Δm/dt assuming dt is constant and equal to 1 for simplicity
        # dm_dt = dm/dt  # Change this if you have a specific dt
        # self.dmdt_data.append(dm_dt) 
        # # self.logger.info(f"Delta m / delta t: {dm_dt}")   
        # # Using the Dynamic Health Assessment
        # if dm_dt <= dyn_low:
        #     self.H_dyn = 1
        # elif dm_dt > dyn_high:
        #     self.H_dyn = 0
        # else:
        #     self.H_dyn = 1 - (dm_dt - dyn_low) / (dyn_high - dyn_low)
        
        # # Ensure health metric is within [0, 1]
        # self.H = min(self.H_alg, self.H_dyn)
        # self.H = max(0, min(self.H, 1))

        # # self.logger.info(f"Health Status: {self.H}")   
        # self.health_data.append(self.H)
        # self.time_steps.append(self.t_now)  # Assuming self.t_now is updated elsewhere to track time
        # self.t_now += 1 

        #### Defining Smooth Function to find Probabilty of Health Measurement Function ####
        if self.H < 0.5:
            x = 2*self.H
            
            # print("in health monitor",x)
            Good = 0.2*self.errfunction(x)
            x= 1-2*self.H
            Bad =  0.2+0.8*self.errfunction(x)
            S= Bad+Good
            good = Good/S
            bad = Bad/S
            good= Good
            bad = 1-Good

        if self.H > 0.5:
            x = 2*self.H-1
            Good = 0.2+0.8*self.errfunction(x)
            x = 2-2*self.H
            Bad =  0.2*self.errfunction(x)
            S= Bad+Good
            good = Good/S
            bad = Bad/S
            good= Good
            bad = 1-Good           
        self.P = np.diag([good, bad])
        
        #### Predict and Updating Health States ####
        self.x = self.L.dot(self.x)/(np.sum(self.x))
        self.x = self.P.dot(self.x)/(np.sum(self.x))

    ''' Without health Monitoring '''
    # def predict(self, z, flag, t):
    #     if self.start < 3:
    #         self.z__ = z
    #         self.z_ = z
    #         self.z = z
    #         self.start += 1
    #     else:
    #         self.z__ = self.z_
    #         self.z_ = self.z
    #         self.z = z
    #     self.L = np.array([[0.9989, 0.0011],
    #                        [0.0011, 0.9989]], dtype=float)

    #     dt = t - self.t_now
    #     self.t_now = t

    #     # Calculate delta m based on the Algebraic Health Assessment formula
    #     dm = np.sqrt((self.z[0] - self.z_[0])**2 + (self.z[1] - self.z_[1])**2)
    #     # self.logger.info(f"Delta m: {dm}")  
    #     self.dm_data.append(dm)      
    #     alg_low = 0.0
    #     alg_high = 0.2

    #     # Using the Algebraic Health Assessment
    #     if dm < alg_low:
    #         self.H_alg = 1
    #     elif dm > alg_high:
    #         self.H_alg = 0
    #     else:
    #         # A quadratic function of Δm between the two thresholds
    #         self.H_alg = 1 - ((dm - alg_low)**2 / (alg_high - alg_low)**2)

    #     dyn_low = 0.0
    #     dyn_high = 1*(1**-7)
    #     # Calculate Δm/dt assuming dt is constant and equal to 1 for simplicity
    #     dm_dt = dm / dt  # Change this if you have a specific dt
    #     self.dmdt_data.append(dm_dt) 
    #     # self.logger.info(f"Delta m / delta t: {dm_dt}")   
    #     # Using the Dynamic Health Assessment
    #     if dm_dt <= dyn_low:
    #         self.H_dyn = 1
    #     elif dm_dt > dyn_high:
    #         self.H_dyn = 0
    #     else:
    #         self.H_dyn = 1 - (dm_dt - dyn_low) / (dyn_high - dyn_low)
        
    #     # Ensure health metric is within [0, 1]
    #     health = min(self.H_alg, self.H_dyn)
    #     # self.H = max(0, min(self.H, 1))

    #     # self.logger.info(f"Health Status: {self.H}")   
    #     self.health_data.append(health)
    #     self.time_steps.append(self.t_now)  # Assuming self.t_now is updated elsewhere to track time
    #     self.t_now += 1 

    #     #### Predicting and Updating States ####
    #     if flag == 1:
    #         self.x = self.L.dot(self.x) / (np.sum(self.x))

    def export_to_excel(self):
        # Create a DataFrame from the health data, Δm data, and time steps
        data = {
            'Time Step': self.time_steps,
            'Health Status': self.health_data,
            'dm': self.dm_data,
            'dmdt': self.dmdt_data,
        }
        df = pd.DataFrame(data)
        
        # Export the DataFrame to an Excel file
        df.to_csv('health_data.txt', index=False)

    # Assuming `hm` is an instance of HealthMonitor
    