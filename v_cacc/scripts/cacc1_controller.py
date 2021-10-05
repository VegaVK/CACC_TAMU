#!/usr/bin/env python3
# One [1] Vehicle Lookup scheme CACC, with packet loss (switchable). Assumes 'LVBag.bag' exists in same folder.
# Consequent bag files are: FV[n]_CACC1.bag, where [n] is vehicle number
# Also assumes platoon is always going in one direction (North/South)
import rospy
import std_msgs
from std_msgs.msg import Time
from bagpy import bagreader
import dbw_mkz_msgs
import delphi_esr_msgs
from sensor_msgs.msg import Image, NavSatFix
from dbw_mkz_msgs.msg import SteeringReport
from delphi_esr_msgs.msg import EsrStatus4
import pymap3d as pm
import numpy as np


def main():
    rospy.init_node('cacc1_controller', anonymous=True)
    accInst=CACCcontroller()
    rospy.spin()

class CACCcontroller():
    def __init__(self):
        ## **** CONFIGURATION *****
        self.PL_Enable=True # Packet losses are enabled, if true
        self.VehNumber=1 # FV number
        self.Direction=1 # +1== North, (-1)==South

        ### ****PARAMETERS ****
        self.Gamma1=1.0 # Packet reception rate (out of 1.0)
        self.CurrentVel=0 # Start condition
        self.timeHeadway=0.45# In seconds
        self.L=1 #in meters
        self.Kp=0.1
        self.Kv=0.2
        self.Ka=0.2
        self.BrakeDecelGain=3 # Additional gain for braking deceleration, >1 for harder decel # TODO check if necessary
        ## **** GPS ORIGIN ****
        self.lat0 = 30.632913
        self.lon0 = -96.481894 # deg
        self.h0 = 54.3     # meters


        ## **** BAG FILES ****
        if self.VehNumber==1:
            self.bLV=bagreader('/home/vamsi/catkin_ws/src/CACC_TAMU/v_cacc/LVBag.bag')
        else:
            prevVehPath='/home/vamsi/catkin_ws/src/CACC_TAMU/v_cacc/FV'+str(self.VehNumber-1)+'_CACC1.bag'
            self.bLV=bagreader(prevVehPath)
        #Position:
        bLVfile1 = self.bLV.message_by_topic(topic='/piksi/navsatfix_best_fix')
        bLVRaw=np.loadtxt(open(bLVfile1, "rb"), delimiter=",", skiprows=1, usecols = (0,7,8,9)) # [Time (s). Lat, Lon, Alt]
        self.bLVPosData=np.zeros((bLVRaw.shape))
        startTimebLV=bLVRaw[0,0]
        for idx in range(bLVRaw.shape[0]):
            self.bLVPosData[idx,0]=bLVRaw[idx,0]-startTimebLV
            tempVal=pm.geodetic2enu(bLVRaw[idx,1], bLVRaw[idx,2], bLVRaw[idx,3], self.lat0, self.lon0, self.h0)
            self.bLVPosData[idx,1]= tempVal[0]
            self.bLVPosData[idx,2]= tempVal[1]
            self.bLVPosData[idx,3]= tempVal[2]
        #Accel:
        bLVfile1 = self.bLV.message_by_topic(topic='/vehicle/brake_info_report')
        bLVRaw=np.loadtxt(open(bLVfile1, "rb"), delimiter=",", skiprows=1, usecols = (0,8)) # [Time (s). Accel_over_ground]
        self.bLVAccelData=np.zeros((bLVRaw.shape))
        startTimebLV=bLVRaw[0,0]
        for idx in range(bLVRaw.shape[0]):
            self.bLVAccelData[idx,0]=bLVRaw[idx,0]-startTimebLV
            self.bLVAccelData[idx,1]= bLVRaw[idx,1]
        #Velocity:
        bLVfile1 = self.bLV.message_by_topic(topic='/vehicle/steering_report')
        bLVRaw=np.loadtxt(open(bLVfile1, "rb"), delimiter=",", skiprows=1, usecols = (0,9)) # [Time (s), speed]
        self.bLVvelData=np.zeros((bLVRaw.shape))
        startTimebLV=bLVRaw[0,0]
        for idx in range(bLVRaw.shape[0]):
            self.bLVvelData[idx,0]=bLVRaw[idx,0]-startTimebLV
            self.bLVvelData[idx,1]= bLVRaw[idx,1]

        ## *********** Other Init    
        self.acc_class=std_msgs.msg.Float32()
        self.acc_pub = rospy.Publisher('x_acc/control_input',std_msgs.msg.Float32,queue_size=2)
        rospy.Subscriber('time_topic', Time, self.timeStore)
        rospy.Subscriber('vehicle/steering_report', SteeringReport, self.velFun)
        rospy.Subscriber('vehicle/brake_info_report', dbw_mkz_msgs.msg.BrakeInfoReport,self.accelfun)
        rospy.Subscriber('piksi/navsatfix_best_fix', NavSatFix,self.posfun)

    def posfun(self,data):
        self.CurrentPos=pm.geodetic2enu(data.latitude,data.longitude,data.altitude, self.lat0, self.lon0, self.h0)
        self.acc_calc()


    def timeStore(self,data):
        # print('timestore')
        self.currentTime=data.data # From time server, starts at zero
    
    def timeIndexSel(self,DataMat):
        for jdx in range(DataMat.shape[0]):
            if (DataMat[jdx,0]-(self.currentTime.secs+self.currentTime.nsecs/10**9)>=0) or (jdx==DataMat.shape[0]-1):
                return jdx
                break
            else:
                continue


    def acc_calc(self):
        # Calculated required acceleration (u) and publish to x_acc topic
        delV=(self.bLVvelData[self.timeIndexSel(self.bLVvelData),1]-self.CurrentVel)
        delX=(self.Direction*(self.bLVPosData[self.timeIndexSel(self.bLVPosData),2]-self.CurrentPos[1])-self.L-self.CurrentVel*self.timeHeadway) # TODO: not sure if currentPos is correct in enu (maybe -ve)
        if self.PL_Enable:
             self.acc_class=self.Gamma1*self.Ka*self.bLVAccelData[self.timeIndexSel(self.bLVAccelData),1]+self.Kv*delV+self.Kv*delX
        else:
            self.acc_class=self.Ka*self.bLVAccelData[self.timeIndexSel(self.bLVAccelData),1]-self.Kv*delV-self.Kv*delX
        if not rospy.is_shutdown():
                    log_Str = ('Published target Controller Output ( CACC_1):',self.acc_class)
                    rospy.loginfo(log_Str)
                    self.acc_pub.publish(self.acc_class)


    def accelfun(self,data):
        # print('accelfun)')
        self.CurrentAccel=data.accel_over_ground
    
    def velFun(self,data3):
        # print('velfun')
        self.CurrentVel=data3.speed


if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")