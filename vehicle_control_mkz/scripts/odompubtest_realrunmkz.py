#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
from vehicle_control_mkz.msg import A9
#from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler as QOE
import utm
import numpy as np
import sys
import pdb
from rclpy.node import Node 
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,QoSDurabilityPolicy
from custom_msgs.msg import positionEstimate
from custom_msgs.msg import orientationEstimate

import threading 
#def QOS
qs= QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
    history=QoSHistoryPolicy.KEEP_ALL
)

class MessageHandle:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.roll =0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 0.0

    def returnStates(self):
        return [self.x, self.y, self.z, self.roll, self.pitch, self.yaw, 
                self.qx, self.qy, self.qz, self.qw]
    
    def callback_p(self,msg,*args):
        #"MKZ REAL"
        #temp = (msg.latitude,msg.longitude)
        temp = utm.from_latlon(msg.latitude, msg.longitude)
        self.x = temp[0]
        self.y = temp[1]


    def callback_o(self,msg,*args):
        # CHANGED FROM SIM FILE######## ROLL,PITCH YAW
        self.roll = np.radians(msg.roll)
        self.pitch = np.radians(msg.pitch)
        self.yaw = np.radians(msg.yaw)
        q=QOE(self.roll,self.pitch,self.yaw)
        self.qx = q[0]
        self.qy = q[1]
        self.qz = q[2]
        self.qw = q[3]


class Sensor_Fusion_Node(Node):
    def __init__(self,mh):
        #Starting sensor
        super().__init__('Sensor_Fusion_Node')
        # define conditional if real and fake 
        #FOR SIM ONLY 
        self.subscription1 = self.create_subscription(orientationEstimate,"/mti/filter/orientation",mh.callback_o, qs)
        self.subscription2 = self.create_subscription(positionEstimate,"/mti/filter/position",mh.callback_p, qs)
        #Publish ODOM TOPICS 
        self.publisher1 = self.create_publisher(A9,'/vehicle/odom2',1)
        self.publisher2 = self.create_publisher(Odometry,'/vehicle/odom1',1)

odomQuat = Odometry()
odomEuler = A9()



def main(args=None):
    
    #initializaiton of message handling node 
    ###############
    #################
    rclpy.init(args=args)
    mh=MessageHandle() 
    argv = sys.argv
    Sf = Sensor_Fusion_Node(mh)
    thread = threading.Thread(target=rclpy.spin, args=(Sf, ), daemon=True)
    thread.start()
    rate = Sf.create_rate(50)
    ##initalize coordinates###
    x = 0.0
    y = 0.0
    z = 0.0
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    x = 0.0
    qy = 0.0
    qz = 0.0
    qw = 0.0
    ####

    while rclpy.ok():
        #### Fill in the messages for quaternion odometry 
        x,y,z,roll,pitch,yaw,qx,qy,qz,qw=mh.returnStates()
        odomQuat.pose.pose.orientation.x = qx
        odomQuat.pose.pose.orientation.y = qy
        odomQuat.pose.pose.orientation.z = qz
        odomQuat.pose.pose.orientation.w = qw
        odomQuat.pose.pose.position.x = x	
        odomQuat.pose.pose.position.y = y	
        odomQuat.pose.pose.position.z = z	
        odomQuat.header.frame_id = "Testing Kennys code translated"
        #odomQuat.header.seq = count, TRANSLATED 
        odomQuat.header.stamp = Sf.get_clock().now().to_msg()
        #### Fill in euler odometry
        odomEuler.x = x
        odomEuler.y = y
        odomEuler.z = z
        odomEuler.roll = roll
        odomEuler.pitch = pitch
        odomEuler.yaw = yaw
        #odomEuler.header.seq = count, TRANSLATED 
        odomEuler.header.stamp = Sf.get_clock().now().to_msg()	
        #### Publish 	
        Sf.publisher2.publish(odomQuat)
        Sf.publisher1.publish(odomEuler)
        #### Sleep and count
        #count += 1
        print ("\t x[utm]={} \n\t y[utm]={} \n\t z[m]={} \n\t roll[rad]={} \n\t pitch[rad]={} \n\t yaw[rad]={} \n\t qx={} \n\t qy={} \n\t qz={} \n\t qw={} \n".format(x,y,z,roll,pitch,yaw,qx,qy,qz,qw))
        rate.sleep()



if __name__ == '__main__':
    main()