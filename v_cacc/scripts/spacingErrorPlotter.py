#!/usr/bin/env python3
# Plots Spacing errors for lossy virtual CACC. Assumes bag files are available
import rospy
import pymap3d as pm
import numpy as np

import matplotlib.pyplot as plt


def main():
    rospy.init_node('error_plotter', anonymous=True)
    accInst=SpacingErrorPlot()
    rospy.spin()

class SpacingErrorPlot ():
    def __init__(self):
       
        self.TotalVehs=4 # including LV
        self.Direction=1 # +1== North, (-1)==South

        ### ****PARAMETERS ****
        self.Gamma1=1.0 # Packet reception rate (out of 1.0)
        self.CurrentVel=0 # Start condition
        self.timeHeadway=0.45# In seconds
        self.L=1 #in meters
        # self.Kp=0.2
        # self.Kv=0.3
        # self.Ka=0.6
        # self.BrakeDecelGain=2 # Unused
        # #Gilbert Model Parameters:
        # self.BadLR=95
        # self.GoodToBad=30
        # self.BadToGood=30
        # self.CurrentGilbertState=1 # 1= Good State; 0 = Bad State
        # self.NextGilbertState=1
        # self.PLFactor=1 
        ## **** GPS ORIGIN ****
        self.lat0 = 30.632913
        self.lon0 = -96.481894 # deg
        self.h0 = 54.3     # meters


        ## **** LOAD BAG FILES ****
        self.Pos=[]
        self.Vel=[]
        self.Accel=[]
        for vIdx in range(self.TotalFollowVehs):
            if vIdx==0:
                # CarBag=bagreader('/home/mkz/mkzbag/2021Oct6/LVBag1.bag')
                CarBag=bagreader('/home/vamsi/Downloads/ATemp/2021Oct6/LVBag1.bag')
            else:
                # prevVehPath='/home/mkz/mkzbag/2021Oct6//FV'+str(vIdx)+'_CACC1.bag'
                prevVehPath='/home/vamsi/Downloads/ATemp/2021Oct6/FV'+str(vIdx)+'_CACC1.bag'
                CarBag=bagreader(prevVehPath)
            Bagfile = CarBag.message_by_topic(topic='/piksi/navsatfix_best_fix')
            TempPos=np.loadtxt(open(Bagfile, "rb"), delimiter=",", skiprows=1, usecols = (0,7,8,9)) # [Time (s). Lat, Lon, Alt]
            Bagfile = CarBag.message_by_topic(topic='/vehicle/brake_info_report')
            TempAccel=np.loadtxt(open(Bagfile, "rb"), delimiter=",", skiprows=1, usecols = (0,8)) # [Time (s). Accel_over_ground]
            Bagfile = CarBag.message_by_topic(topic='/vehicle/steering_report')
            TempSpeed=np.loadtxt(open(Bagfile, "rb"), delimiter=",", skiprows=1, usecols = (0,9)) # [Time (s), speed]
            startTime=TempPos[0,0]
            for idx in range(TempPos.shape[0]):
                TempPos[idx,0]=TempPos[idx,0]-startTime
                tempVal=pm.geodetic2enu(TempPos[idx,1], TempPos[idx,2], TempPos[idx,3], self.lat0, self.lon0, self.h0) # Convert to ENU
                TempPos[idx,1]= tempVal[0] 
                TempPos[idx,2]= tempVal[1] # North, (X), of interest.
                TempPos[idx,3]= tempVal[2]
            startTime=TempSpeed[0,0]
            for idx in range(TempSpeed.shape[0]):
                TempSpeed[idx,0]=TempSpeed[idx,0]-startTime
            startTime=TempAccel[0,0]
            for idx in range(TempAccel.shape[0]):
                TempAccel[idx,0]=TempAccel[idx,0]-startTime
            self.Pos.append(TempPos)
            self.Vel.append(TempSpeed)
            self.Accel.append(TempAccel)
        self.plotter()

    def plotter(self,data):
        for vIdx in range(1,self.TotalFollowVehs):
            vecLength=min(self.Pos[vIdx].shape[0], self.Pos[vIdx-1].shape[0])
            delX=np.zeros((vecLength,1))
            for idx in range(vecLength):
                delX[idx]=(self.Direction*(self.Pos[vIdx-1][idx,2]-self.Pos[vIdx][idx,2])-self.L-self.Vel[vIdx][self.timeIndexSel(self.Vel[vIdx],self.Pos[vIdx][0,0])*self.timeHeadway)
        
            plt.plot(self.Pos[vIdx][:,0], delX)
        plt.ylabel('Spacing Error')
        plt.xlable('Time (s)')
        plt.show()
    
    def timeIndexSel(self,DataMat,targetTime):
        for jdx in range(DataMat.shape[0]):
            if (DataMat[jdx,0]-targetTime>=0) or (jdx==DataMat.shape[0]-1):
                return jdx
                break
            else:
                continue


if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")