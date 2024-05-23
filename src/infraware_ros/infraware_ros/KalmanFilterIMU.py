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
        # self.x = np.array([[0], [0.], [0.],[0.],[0.]], dtype=float)
        # self.x_size = np.size(self.x)
        # self.P = np.diag([0.5, 0.5, 0.5,0.5,0.5])
        # self.F = np.zeros((self.x_size, self.x_size), dtype=float)
        # self.K = np.zeros((self.x_size, self.x_size), dtype=float)
        # self.Q = np.diag([0.1,0.1,0.1,0.1,0.1])
        # self.S = np.zeros((2, 2), dtype=float)
        # self.y = np.zeros((2,1), dtype=float)
        # self.R = np.diag([0.5,  0.5])
        # self.H = np.array([[1, 0, 0,0,0],
        #                    [0, 1, 0,0,0]], dtype=float)



        self.x = np.array([[0], [0.], [0.]], dtype=float)
        self.x_size = np.size(self.x)
        self.P = np.diag([0.5, 0.5, 0.5])
        self.F = np.zeros((self.x_size, self.x_size), dtype=float)
        self.K = np.zeros((self.x_size, self.x_size), dtype=float)
        self.Q = np.diag([0.1,0.1,0.01])
        self.S = np.zeros((2, 2), dtype=float)
        self.y = np.zeros((2,1), dtype=float)
        self.R = np.diag([0.5,  0.5])
        self.H = np.array([[1, 0, 0],
                           [0, 1, 0]], dtype=float)
 
 
 
 
    def dynamic_model_slope(self,state,Vx,delta): # state : x y yaw vx vy r(yaw rate)
        Cr = 174956.65
        Cf = 29581.16
        Iz = 1539.45
        m = 1715.27
        Lf = 1.33
        Lr = 1.52
        if Vx >0:
            beta = math.atan(np.tan(delta)*Lr/(Lr+Lf))
            print("beta",beta)
            f = np.array([[(Vx*np.cos(state[2]+beta)).item()],
                          [(Vx*np.sin(state[2]+beta)).item()],
                          [(Vx*np.sin(beta)/Lr).item()]])
            # f = np.array([[(Vx*np.cos(state[2])-state[3]*np.sin(state[2])).item()],
            #           [ (Vx*np.sin(state[2])+state[3]*np.cos(state[2])).item()],
            #           [ (state[4]).item()],
            #           [(-state[3]*(Cf*np.cos(delta)+Cr)/(m*Vx) + state[4]*(-Lf*Cf*np.cos(delta)+Cr*Lr**2)/(Iz*Vx)+delta*Cf*np.cos(delta)/m).item()],
            #           [((state[3]*(-Lf*Cf*np.cos(delta)+Cr*Lr*2)/(m*Vx)-state[3]*Vx)-state[4]*(Cf*np.cos(delta)*Lf**2+Cr*Lr**2)/(Iz*Vx)+delta*Lf*Cf*np.cos(delta)/Iz).item()]])
        else:
            f = state
        return f       
    

    def dynamic_model(self,y_,vx,delta,dt):
        y_ = np.reshape(y_,(self.x_size,1))
        k1 = self.dynamic_model_slope(y_,vx,delta)
        k2 = self.dynamic_model_slope(y_+ dt*k1/2,vx,delta)
        k3 = self.dynamic_model_slope(y_+ dt*k2/2,vx,delta)
        k4 = self.dynamic_model_slope(y_+ dt*k3,vx,delta)
        y = np.reshape(y_+ dt*(k1+2*k2+2*k3+k4)/6,(self.x_size,1))
        
        return y
    
    def dynamic_for_jacob(self,y_,vx,delta,dt): # output needs to be 1-D for optimize.approx_fprime
        y_ = np.reshape(y_,(self.x_size,1))
        k1 = self.dynamic_model_slope(y_,vx,delta)
        print("HI",k1)
        k2 = self.dynamic_model_slope(y_+ dt*k1/2,vx,delta)
        k3 = self.dynamic_model_slope(y_+ dt*k2/2,vx,delta)
        k4 = self.dynamic_model_slope(y_+ dt*k3,vx,delta)
        y = np.reshape(y_+ dt*(k1+2*k2+2*k3+k4)/6,(self.x_size,1))
        # y_flat =[y[0][0],y[1][0],y[2][0],y[3][0],y[4][0]]
        y_flat =[y[0][0],y[1][0],y[2][0]]

        return y_flat
         
    
    def Jacobian_F(self,x_,vx,delta,dt):
        eps = np.sqrt(np.finfo(float).eps)
        # x = [x_[0][0],x_[1][0],x_[2][0],x_[3][0],x_[4][0]]
        # EPS = [np.sqrt(200)*eps, np.sqrt(200)*eps, eps,eps,eps]
        x = [x_[0][0],x_[1][0],x_[2][0]]
        EPS = [np.sqrt(200)*eps, np.sqrt(200)*eps, eps]

        Jf = optimize.approx_fprime(x, self.dynamic_for_jacob, EPS,vx,delta,dt)
        Jf = np.reshape(Jf,(self.x_size,self.x_size))
        return Jf


    ##############################################
    ### Measurement Model                      ###
    ##############################################
    def set_state(self, z):
        self.x[0] = z[0]
        self.x[1] = z[1]
        self.new = False


    def EKF_predict(self,vx,delta,dt):
        if vx > 0:
        
            self.F = self.Jacobian_F(self.x,vx,delta,dt)
            print("self.x",self.x,self.F,self.K,vx,delta)
            self.x = self.dynamic_model(self.x,vx,delta,dt)
        else:
            self.x = self.x
            print("HI")
    def EKF_update(self,vx,z):
        if vx > 0:
            self.P = self.F.dot(self.P).dot(self.F.T) + self.Q
            self.S = self.R + self.H.dot(self.P).dot(self.H.T)

            self.K = self.P.dot(self.H.T).dot(np.linalg.pinv(self.S))
            self.x = self.x + self.K.dot(z.reshape(2,1)-self.H.dot(self.x))
            self.P = (np.eye(self.x_size) - self.K.dot(self.H)).dot(self.P)



   


