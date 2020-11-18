#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.

import rospy
import dbw_mkz_msgs.msg
from geometry_msgs.msg import TwistStamped
import std_msgs
import numpy as np
import time
import scipy.io as sio
from scipy import interpolate

# ==== Controller Parameters:
global d #Standstill Distance - Not used here, for ACC/CACC only
global Ki
global Kp
global Kv
global hw # Time headway - Not used here
global clockStore # for getting time step

d=10
Ki=0.05
Kp=1
Kd=0.5

def accelPub(DesiredVel):
    global CURRENTVEL
    global CURRENTACCEL
    global Ki
    global Kp
    global Kv
    global CLOCKSTORE
    global PREV_ERROR_V

    targetAccelPub = rospy.Publisher('x_acc/control_input',std_msgs.msg.Float32, queue_size=10)
    r_wh=0.2413 # Radius of Wheel
    m=1800 # Approx weight of car
    # Calculate desired accel:
    delT=time.time()-CLOCKSTORE
    CLOCKSTORE=time.time()
    errorV=DesiredVel-CURRENTVEL
    if delT==0.0: # To avoid div by 0
        errorAccelIMU=0
        errorAccelDeriv=0
    else:
        errorAccelIMU = (errorV-PREV_ERROR_V)/delT - CURRENTACCEL
    #    errorAccelDeriv=(errorV-PREV_ERROR_V)/delT
    if abs(errorV)<=1:# Only use PID if errors are small; else use PD
        errorIntegral=0.5*delT*(errorV+PREV_ERROR_V)
    else:
        errorIntegral=0.0
    PREV_ERROR_V=errorV
    targetAccel=Kp*errorV+Kv*errorAccelIMU+Ki*errorIntegral
    targetAccelPub.publish(targetAccel)
    rospy.loginfo_once(rospy.get_caller_id()+'Target Accleration Published')



def velfun(data,DesiredVel):
    global CURRENTVEL
    CURRENTVEL=data.twist.linear.x
    accelPub(DesiredVel)

def accelfun(data):
    global CURRENTACCEL
    CURRENTACCEL=data.accel_over_ground.data


def listener():
    # Subscribe to external ACC/CACC controller
    inputTemp=float(input("Please enter the desired velocity in MPH: ")
    DesiredVel=inputTemp*0.44704
    rospy.init_node('vel_tracker', anonymous=True)
    rospy.Subscriber('/vehicle/twist', TwistStamped,velfun,(DesiredVel))
    rospy.Subscriber('/vehicle/brake_info_report', dbw_mkz_msgs.msg.BrakeInfoReport,accelfun)
    rospy.spin()

if __name__=='__main__':
    try:
        global CLOCKSTORE
        CLOCKSTORE=time.time()
        listener()
    except rospy.ROSInterruptException:
        pass