 #!/usr/bin/env python
import numpy as np
import math
from scipy.integrate import odeint,solve_ivp
from scipy import optimize

class KalmanFilterIMU:

    def __init__(self):
        # Each KF object gets a unique object ID
        self.ID = 0
        self.new = True
        self.flag_ = 5

        # Kalman Filter parameters
        self.x = np.array([[0], [0.], [-1.57],[0.],[0.],[0.]], dtype=float)
        self.x_size = np.size(self.x)
        self.P = np.diag([3., 2., 0.01, 0.1,0.1,0.01])
        self.F = np.zeros((self.x_size, self.x_size), dtype=float)
        self.K = np.zeros((self.x_size, self.x_size), dtype=float)
        self.Q = self.P
        self.S = np.zeros((2, 2), dtype=float)
        self.y = np.zeros((2,1), dtype=float)
        self.R = np.diag([0.5,  0.5])
        self.H = np.array([[1, 0, 0,0,0,0],
                           [0, 1, 0,0,0,0]], dtype=float)

 
 
 
 
    def dynamic_model_slope(self,state,Vx,delta): # state : x y yaw vx vy r(yaw rate)
        Cr = 174956.65
        Cf = 29581.16
        Iz = 1539.45
        m = 1715.27
        Lf = 1.33
        Lr = 1.52
        f = np.array([[ (state[3]*np.cos(state[2])-state[4]*np.sin(state[2])).item()],
                      [ (state[3]*np.sin(state[2])+state[4]*np.cos(state[2])).item()],
                      [ (state[5]).item()  ],
                      [ Vx],
                      [(-state[3]*state[5]-state[4]*(Cf*np.cos(delta)+Cr)/(m*state[3])+ state[5]*(-Lf*Cf*np.cos(delta)+Cr*Lr**2)/(Iz*state[3])+state[5]*Cf*np.cos(delta)/m).item()],
                      [(state[4]**(-Lf*Cf*np.cos(delta)+Cr*Lr**2)/(m*state[3])-state[3]*state[5]-state[5]*(Cf*np.cos(delta)*Lf**2+Cr*Lr**2)/(Iz*state[3])+state[5]*Lf*Cf*np.cos(delta)/Iz).item]])
        
        return f       
    

    def dynamic_model(self,y_,vx,delta):
        y_ = np.reshape(y_,(self.x_size,1))
        k1 = self.dynamic_model_slope(y_,vx,delta)
        #print(k1)
        k2 = self.dynamic_model_slope(y_+ self.dt*k1/2),vx,delta
        k3 = self.dynamic_model_slope(y_+ self.dt*k2/2,vx,delta)
        k4 = self.dynamic_model_slope(y_+ self.dt*k3,vx,delta)
        y = np.reshape(y_+ self.dt*(k1+2*k2+2*k3+k4)/6,(self.x_size,1))

        return y
    
    def dynamic_for_jacob(self,y_,vx,delta): # output needs to be 1-D for optimize.approx_fprime
        y_ = np.reshape(y_,(self.x_size,1))
        k1 = self.dynamic_model_slope(y_,vx,delta)
        #print(k1)
        k2 = self.dynamic_model_slope(y_+ self.dt*k1/2,vx,delta)
        k3 = self.dynamic_model_slope(y_+ self.dt*k2/2,vx,delta)
        k4 = self.dynamic_model_slope(y_+ self.dt*k3,vx,delta)
        y = np.reshape(y_+ self.dt*(k1+2*k2+2*k3+k4)/6,(self.x_size,1))
        y_flat =[y[0][0],y[1][0],y[2][0]]
        return y_flat
         
    
    def Jacobian_F(self,x_,vx,delta):
        eps = np.sqrt(np.finfo(float).eps)
        v = np.zeros((self.x_size,1))
        x = [x_[0][0],x_[1][0],x_[2][0]]
        Jf = optimize.approx_fprime(x, self.dynamic_for_jacob, [np.sqrt(200)*eps,  eps, 0.001*eps],vx,delta)
        Jf = np.reshape(Jf,(self.x_size,self.x_size))
        return Jf


    ##############################################
    ### Measurement Model                      ###
    ##############################################
    def set_state(self, z):
        self.x[0] = z[0]
        self.x[1] = z[1]
        self.new = False


    def EKF_(self,vx,delta,z):
        self.F = self.Jacobian_F(self.x,vx,delta)
        self.x = self.dynamic_model(self.x,np.zeros((self.x_size,1)))


        self.P = self.F.dot(self.P).dot(self.F.T) + self.Q
        self.S = self.R + self.H.dot(self.P).dot(self.H.T)

        self.K = self.P.dot(self.H.T).dot(np.linalg.pinv(self.S))
        self.x = self.x + self.K.dot(z-self.H.dot(self.x))
        self.P = (np.eye(self.x_size) - self.K.dot(self.H)).dot(self.P)


   


