#!/usr/bin/env python
# Software License Agreement (BSD License)
#

## Just read rosbag file, generate an array with throttle,vel and accel
import scipy.io as sio
import rospy
import dbw_mkz_msgs.msg 
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import pickle as pl 


class plotDat:
    pedal=dbw_mkz_msgs.msg.ThrottleReport()
    Accel=dbw_mkz_msgs.msg.BrakeInfoReport()
    Vel=dbw_mkz_msgs.msg.SteeringReport()
    SavArr=np.array([0,0,0,0])

def pedalfun(data):
    plotDat.pedal.pedal_cmd=data.pedal_cmd
    plotDat.pedal.pedal_output=data.pedal_output
def builder():
    # X =throttle_output, y = vel, Z= accel, z2=throttle_cmd
    temp=np.array([plotDat.pedal.pedal_output,plotDat.Vel.speed,plotDat.Accel.accel_over_ground,plotDat.pedal.pedal_cmd])
    plotDat.SavArr=np.vstack((plotDat.SavArr,temp))
    #print(plotDat.pedal.pedal_cmd)
def accelfun(data):
    plotDat.Accel.accel_over_ground=data.accel_over_ground
def velfun(data):
    plotDat.Vel.speed=data.speed
def listener():
    rospy.init_node('throttlemap_plotter', anonymous=True)
    rate = rospy.Rate(100) # 50hz
    while not rospy.is_shutdown():
        rospy.Subscriber('vehicle/throttle_report', dbw_mkz_msgs.msg.ThrottleReport, pedalfun)
        rospy.Subscriber('vehicle/brake_info_report',dbw_mkz_msgs.msg.BrakeInfoReport,accelfun)
        rospy.Subscriber('vehicle/steering_report',dbw_mkz_msgs.msg.SteeringReport,velfun)
        #rospy.loginfo(rospy.get_caller_id()+'Msg received')
        builder()
        rate.sleep()

if __name__=='__main__':
    try:
        listener()
        #Save file here
        sio.savemat('ExpData1.mat',{'EngineData':plotDat.SavArr})
    except rospy.ROSInterruptException:

        pass