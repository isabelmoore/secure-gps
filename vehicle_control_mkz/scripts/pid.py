#!/usr/bin/env python
class PID:
    def __init__(self,kp=1,ki=0.0,kd=0.0,satLower=-100., satUpper=100.):
        self.errorNow = 0.0
        self.errorTot = 0.0
        self.errorPrev = 0.0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.satLower = satLower
        self.satUpper = satUpper
    
    def update(self,error):
        #### set previous error, current error, and accumulated error
        self.errorPrev = self.errorNow
        self.errorNow = error
        self.errorTot += error
        self.errorTot = self.satValues(self.errorTot,self.satLower,self.satUpper)

    def errorTotalReturn(self):
        return self.errorTot

    def setGains(self,kp,ki,kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def setLims(self,satLower,satUpper):
        self.satLower = satLower
        self.satUpper = satUpper

    def satValues(self,value,satLower, satUpper):
        if value >= satUpper:
            return satUpper
        elif value <= satLower:
            return satLower
        else:
            return value

    def computeControl(self):
        #### Bound total error
        kp = self.kp
        ki = self.ki
        kd = self.kd
        #self.errorTot = self.satValues(self.errorTot,self.satLower,self.satUpper) 
        #### Compute control input
        u = kp*self.errorNow + ki*self.errorTot + kd*(self.errorNow - self.errorPrev)
        return u


