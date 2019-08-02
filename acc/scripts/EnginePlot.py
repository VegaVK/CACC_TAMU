#!/usr/bin/env python
# Software License Agreement (BSD License)
#

## plotting accel vs throttle pedal

import rospy
import dbw_mkz_msgs.msg 
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
class plotDat:
    pedal=dbw_mkz_msgs.msg.ThrottleReport()
    Accel=dbw_mkz_msgs.msg.BrakeInfoReport()
    aIdx=0
    pIdx=0
plt.grid(True)
def pedalfun(data):
    plotDat.pedal.pedal_cmd=data.pedal_output
def plotter():
    print(plotDat.pedal.pedal_cmd)
    print(plotDat.Accel.accel_over_ground)
    plt.plot(plotDat.pedal.pedal_cmd,plotDat.Accel.accel_over_ground,'r+')
    plt.draw()
    plt.pause(0.00000001)
def accelfun(data):
    plotDat.Accel.accel_over_ground=data.accel_over_ground
def listener():
    rospy.init_node('throttlemap_plotter', anonymous=True)
    rate = rospy.Rate(50) # 10hz
    while not rospy.is_shutdown():
        rospy.Subscriber('vehicle/throttle_report', dbw_mkz_msgs.msg.ThrottleReport, pedalfun)
        rospy.Subscriber('vehicle/brake_info_report',dbw_mkz_msgs.msg.BrakeInfoReport,accelfun)
        rospy.loginfo(rospy.get_caller_id()+'Msg received')
        plotter()
        print('supposed to have drawn')
        rate.sleep()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass