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
        self.H_dyn = 1
        self.a =0
        self.dy=0
        self.logger = logger
        self.health_data = []
        self.time_steps = []
        self.dm_data = []
        self.dmdt_data = []
        self.domega_data = []

        self.start = 0
        # self.plot_timer = self.create_timer(30, self.plot_all_health_data)  # Every 30 seconds

    def errfunction(self,x):
        erf = 0.5*(1+math.erf(3*x))
        erf = 0.5*(1+math.erf(6*x-3))
        return erf

    def calculate_angle(self, a, b, c):
        """Calculate the angle ABC (in radians) formed by points A, B, and C."""
        ba = a - b
        bc = c - b
        ba = ba.flatten()  # Ensure ba is a 1D array
        bc = bc.flatten()  # Ensure bc is a 1D array
        norm_ba = np.linalg.norm(ba)
        norm_bc = np.linalg.norm(bc)
        
        if norm_ba == 0 or norm_bc == 0:
            return 0  # Or handle the case as needed

        cosine_angle = np.dot(ba, bc) / (norm_ba * norm_bc)
        cosine_angle = np.clip(cosine_angle, -1.0, 1.0)  # Clip to avoid numerical issues
        angle = np.arccos(cosine_angle)
        return angle

    ''' Health Monitoring'''

    def predict(self,z ,flag, t): 
        if self.start <3 :
            self.z___ = z
            self.z__ = z
            self.z_= z
            self.z = z
            self.start += 1
        else:
            self.z___ = self.z__
            self.z__ = self.z_
            self.z_ = self.z
            self.z = z
        self.L = np.array([[0.9989,  0.0011],
                           [0.0011,  0.9989]], dtype=float)
        
        ### Defining Health Measurement Function ####
        # if (flag == 3): 
        #     ax = self.z[0]-2*self.z_[0]+ self.z__[0]
        #     ay = self.z[1]-2*self.z_[1]+ self.z__[1]
        #     self.a = np.sqrt(ax**2 + ay**2)
        #     self.H_1 = 1- 1*np.sign(self.a)* self.a
        #     if self.H_1 < 0 :
        #         self.H_1 = 0
        
        #     self.dy = self.z[0]-self.z_[0]
        #     self.H_2 = 1-np.sign(self.dy)*self.dy/4
        #     if self.H_2 < 0:
        #         self.H_2 = 0
        #     self.H = min(self.H_1,self.H_2)
        # else:
        #     self.H = 1
        # Assuming some predefined thresholds based on system requirements
        if (flag == 2) or (flag == 3):
        # if (flag == 2):

            dt = t - self.t_now
            self.t_now = t

            # Calculate delta m based on the Algebraic Health Assessment formula
            dm = np.sqrt((self.z[0] - self.z_[0])**2 + (self.z[1] - self.z_[1])**2)
            # self.logger.info(f"Delta m: {dm}")  
            self.dm_data.append(dm)      
            alg_low = 0.0
            alg_high = 0.5

            # # Using the Algebraic Health Assessment
            if dm <= alg_low:
               self.H_alg = 0
            elif dm > alg_high:
               self.H_alg = 0
            else:
               # A quadratic function of Δm between the two thresholds
               self.H_alg = 1 - ((dm - alg_low)**2 / (alg_high - alg_low)**2)

            dyn_low = 0.0
            dyn_high = 7*(1**-9)
            # Calculate Δm/dt assuming dt is constant and equal to 1 for simplicity
            dm_dt = dm/dt  # Change this if you have a specific dt
            self.dmdt_data.append(dm_dt) 
            # self.logger.info(f"Delta m / delta t: {dm_dt}")   
            # Using the Dynamic Health Assessment
            if dm_dt <= dyn_low:
                self.H_dyn = 0
            elif dm_dt > dyn_high:
                self.H_dyn = 0
            else:
                self.H_dyn = 1 - (dm_dt - dyn_low) / (dyn_high - dyn_low)
            current_angle = self.calculate_angle(self.z__, self.z_, self.z)
            previous_angle = self.calculate_angle(self.z___, self.z__, self.z_)
            
            angle_change = current_angle - previous_angle

            delta_low = -0.04
            delta_high = 0.04

            # Apply bounds to H_4
            if angle_change <= delta_low:
                self.H_4 = 0
            elif angle_change >= delta_high:
                self.H_4 = 0
            elif angle_change == 0:
                self.H_4 = 1
            else:
                self.H_4 = 1 - (abs(angle_change)**2 / (delta_high)**2)
            
            # self.H = self.H_4

            self.H = min(self.H_alg, self.H_dyn, self.H_4)
            # Ensure health metric is within [0, 1]
            self.H = max(0, min(self.H, 1))

            self.health_data.append(self.H)
            self.time_steps.append(self.t_now)  # Assuming self.t_now is updated elsewhere to track time
            self.domega_data.append(angle_change)  

            self.t_now += 1 

            #### Defining Smooth Function to find Probabilty of Health Measurement Function ####
            if self.H <= 0.5:
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

            else:
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


    def export_to_excel(self):
        # Check if there are any entries to write
        if len(self.health_data) > 0:
            self.logger.info(f"Attempting to write {len(self.health_data)} entries to the file.")
            try:
                # Create a DataFrame from the health data, Δm data, and time steps
                data = {
                    'Time Step': self.time_steps,
                    'Health Status': self.health_data,
                    'dm': self.dm_data,
                    'dmdt': self.dmdt_data,
                    'domega': self.domega_data,
                }
                df = pd.DataFrame(data)
                
                # Export the DataFrame to an Excel file
                df.to_csv('health_data.txt', index=False)
                self.logger.info("Data successfully written to file.")
            except Exception as e:
                self.logger.info(f"Failed to write data to file: {e}")
        else:
            self.logger.info("No entries to write to file.")

    # Assuming `hm` is an instance of HealthMonitor
    