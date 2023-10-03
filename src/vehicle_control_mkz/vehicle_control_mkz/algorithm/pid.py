#!/usr/bin/env python
from vehicle_control_mkz.utility.shared_functions import sat_values

class PID:
    def __init__(self,kp=1, ki=0.0, kd=0.0, sat_lower=-100. , sat_upper=100. ):
        # Initialize all errors to 0.0
        self.error_now = 0.0
        self.error_tot = 0.0
        self.error_prev = 0.0
        
        # Initialize gains
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        # Initialize saturation values
        self.sat_lower = sat_lower
        self.sat_upper = sat_upper
    
    def update(self, error):
        # Update errors
        self.error_prev = self.error_now
        self.error_now = error
        self.error_tot += error
        
        # Bound total error
        self.error_tot = sat_values(self.error_tot,self.sat_lower,self.sat_upper)

    def get_error_total(self):
        return self.error_tot

    # Update gains
    def set_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def set_lims(self, sat_lower, sat_upper):
        self.sat_lower = sat_lower
        self.sat_upper = sat_upper

    # Get control input
    def compute_control(self):
        # Bound total error from integral term
        #self.errorTot = self.satValues(self.errorTot,self.sat_lower,self.sat_upper) 
        
        # Compute control input
        u = self.kp*self.error_now + self.ki*self.error_tot + self.kd*(self.error_now - self.error_prev)
        
        return u


