#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
## Standard ACC implementation, with no use of acceleration from radar, or otherwise
## Also creates image for radar target in /Thermal_Panorama if available
import rospy
import std_msgs
import dbw_mkz_msgs
import delphi_esr_msgs
from sensor_msgs.msg import Image
from dbw_mkz_msgs.msg import SteeringReport
from delphi_esr_msgs.msg import EsrStatus4
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError

def main():
    rospy.init_node('acc_controller', anonymous=True)
    accInst=ACCcontroller()
    rospy.spin()


class ACCcontroller():
    def __init__(self):
        self.CurrentVel=0 # Start condition
        self.timeHeadway=0.5# In seconds
        self.L=15# in meters
        self.Kp=0.1
        self.Kv=0.1
        self.BrakeDecelGain=1.0 # Additional gain for braking deceleration, >1 for harder decel
        self.CipvID=0
        self.PrevTrackID=self.CipvID
        self.CamXOffset=2.36#=93 inches, measured b/w cam and Rdr, in x direction
        self.CamZoffset=1 # Roughly 40 inches
        self.font=cv2.FONT_HERSHEY_SIMPLEX 
        self.bridge=CvBridge()
        self.acc_class=std_msgs.msg.Float32()
        self.RadarTargetData=delphi_esr_msgs.msg.EsrTrack()
        self.RadarTargetOpt1Data=delphi_esr_msgs.msg.EsrTrack()
        self.RadarTargetOpt2Data=delphi_esr_msgs.msg.EsrTrack()
        self.acc_pub = rospy.Publisher('x_acc/control_input',std_msgs.msg.Float32,queue_size=2)
        self.image_pub=rospy.Publisher("accImage",Image, queue_size=2) 
        rospy.Subscriber('vehicle/steering_report', SteeringReport, self.velFun)
        rospy.Subscriber('parsed_tx/radarstatus4', EsrStatus4, self.trackSelector)
        rospy.Subscriber('parsed_tx/radartrack', delphi_esr_msgs.msg.EsrTrack,  self.acc_calc)
        # rospy.Subscriber('Thermal_Panorama', Image,  self.plotter)
            # rospy.loginfo(rospy.get_caller_id()+'x_acc received')
    # acc_calc uses the identified track and current velocity to calculate controller effort(accel) and publishes it
    
    def acc_calc(self,data):
        if  data.track_id==self.TargetOpt1:
            self.RadarTargetOpt1Data=data
        elif data.track_id==self.TargetOpt2:
            self.RadarTargetOpt2Data=data
        
        if  data.track_id==self.CipvID:
            self.RadarTargetData=data
            # print(CipvID)
            self.meas_range=data.track_range # x_{i-1} -x_i
            # print('Range:')
            # print(self.meas_range)
            self.meas_range_rate=data.track_range_rate
            # print('RangeRate:')
            # print(self.meas_range_rate)
            
            
            targetAccel=-self.Kp*(-self.meas_range+self.L+self.CurrentVel*self.timeHeadway)-self.Kv*(-self.meas_range_rate)
            if targetAccel<0:
                self.acc_class=targetAccel*self.BrakeDecelGain
            else:
                self.acc_calc=targetAccel
            print('Meas Dist:')
            MeasHw=self.meas_range
            print(MeasHw)
            print('Des Dist:')
            DesHw=self.L+self.CurrentVel*self.timeHeadway
            print(DesHw)

            if not rospy.is_shutdown():
                        log_Str = ('Published target Controller Output ( Pure ACC):',self.acc_class)
                        # rospy.loginfo(log_Str)
                        self.acc_pub.publish(self.acc_class)
        elif self.CipvID==0:
            self.acc_class=0 # No controller output if not found
            rospy.logwarn('No Target, maintain zero acceleration')
            # rospy.logwarn('No ACC Target found')
            # if not rospy.is_shutdown():
            #             log_Str = ('Published target Controller Output ( Pure ACC):',acc_class)
            #             rospy.loginfo(log_Str)
            #             acc_pub.publish(acc_class)
    # trackSelector Identifies the relevant track for ACC
    
    def trackSelector(self,data2):
        from dbw_mkz_msgs.msg import SteeringReport
        from delphi_esr_msgs.msg import EsrStatus4
        self.PrevTrackID=self.CipvID
        self.TargetOpt1=data2.path_id_acc
        self.TargetOpt2=data2.path_id_acc_stat
        
        # print(data2.path_id_acc)
        if (not (data2.path_id_acc==0) or not(data2.path_id_acc_stat==0)): # pick one or the other
            
            if data2.path_id_acc==0:
                self.CipvID=data2.path_id_acc_stat-1
            else:
                self.CipvID=data2.path_id_acc-1
        else:
            self.CipvID=0 # Or else, No track found
        
        if self.PrevTrackID!=self.CipvID:
            rospy.logwarn('ACC Target Changed')

    # Callback 3 is for obtaining ego vehicle velocity
    
    def velFun(self,data3):
        self.CurrentVel=data3.speed
        # print (CurrentVel)

    def plotter(self,data):
        # If thermal panorama is available, uses the current radar track and plots it
        LocalImage=self.bridge.imgmsg_to_cv2(data, "rgb8")
        # For Target:
        temp2=np.divide(self.CamZoffset,self.RadarTargetData.track_range+self.CamXOffset)
        RadarAnglesV=np.abs(np.degrees(np.arctan(temp2.astype(float)))) #will always be negative, so correct for it
        CameraX=np.dot(self.RadarTargetData.track_angle,(LocalImage.shape[1]/190)) + LocalImage.shape[1]/2 
        CameraY=np.dot(RadarAnglesV,(LocalImage.shape[0]/39.375)) +512/2 # Number of pixels per degree,adjusted for shifting origin from centerline to top left
        LocalImage=cv2.circle(LocalImage, (int(CameraX),int(CameraY)), 12, [0,165,255],3)
        LocalImage=cv2.putText(LocalImage,str('T'),(int(CameraX),int(CameraY)),self.font,1,(255,105,180),2)
        # For Option1:
        temp2=np.divide(self.CamZoffset,self.RadarTargetOpt1Data.track_range+self.CamXOffset)
        RadarAnglesV=np.abs(np.degrees(np.arctan(temp2.astype(float)))) #will always be negative, so correct for it
        CameraX=np.dot(self.RadarTargetOpt1Data.track_angle,(LocalImage.shape[1]/190)) + LocalImage.shape[1]/2 
        CameraY=np.dot(RadarAnglesV,(LocalImage.shape[0]/39.375)) +512/2 # Number of pixels per degree,adjusted for shifting origin from centerline to top left
        LocalImage=cv2.circle(LocalImage, (int(CameraX),int(CameraY)), 12, [0,165,255],3)
        LocalImage=cv2.putText(LocalImage,str('O1'),(int(CameraX),int(CameraY)),self.font,1,(255,105,180),2)
         # For Option2:
        temp2=np.divide(self.CamZoffset,self.RadarTargetOpt2Data.track_range+self.CamXOffset)
        RadarAnglesV=np.abs(np.degrees(np.arctan(temp2.astype(float)))) #will always be negative, so correct for it
        CameraX=np.dot(self.RadarTargetOpt2Data.track_angle,(LocalImage.shape[1]/190)) + LocalImage.shape[1]/2 
        CameraY=np.dot(RadarAnglesV,(LocalImage.shape[0]/39.375)) +512/2 # Number of pixels per degree,adjusted for shifting origin from centerline to top left
        LocalImage=cv2.circle(LocalImage, (int(CameraX),int(CameraY)), 12, [0,165,255],3)
        LocalImage=cv2.putText(LocalImage,str('O2'),(int(CameraX),int(CameraY)),self.font,1,(255,105,180),2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(LocalImage, "bgr8"))

if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")