#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.

import rospy
import dbw_mkz_msgs.msg
from geometry_msgs.msg import TwistStamped
from rospy.core import rospydebug
import std_msgs
import numpy as np
import scipy.io as sio
from scipy import interpolate

def main():
    # This only runs if velTracker is called by itself as a script
    someInst= velTrackDirect()

class velTrackDirect():
    # In case this script is called independently
    def __init__(this):
        this.CurrentVel=0.0 # in m/s
        this.CurrentAccel=0.0
        rospy.init_node('vel_tracker', anonymous=True)
            # Subscribe to external ACC/CACC controller
        inputTemp=float(input("Please enter the desired velocity in MPH: "))
        DesiredVelInput=inputTemp*0.44704 # Converts to m/s
        this.veltrack_inst=velTrackClass(DesiredVelInput)
        this.targetAccelPub = rospy.Publisher('x_acc/control_input',std_msgs.msg.Float32, queue_size=10)
        rospy.Subscriber('/vehicle/twist', TwistStamped,this.velfun)
        rospy.Subscriber('/vehicle/brake_info_report', dbw_mkz_msgs.msg.BrakeInfoReport,this.accelfun)
        rospy.spin()

    def velfun(this,data):
        this.CurrentVel=data.twist.linear.x
        this.veltrack_inst.updateCurrVel(this.CurrentVel)

    def accelfun(this,data):
        this.CurrentAccel=data.accel_over_ground
        this.veltrack_inst.updateCurrAccel(this.CurrentAccel)
        targetAccel=this.veltrack_inst.getTargetAccelForll()
        this.targetAccelPub.publish(targetAccel)
        rospy.loginfo_once(rospy.get_caller_id()+'Target Accleration Published')


class velTrackClass():
    import time
    def __init__(self,desVelInput):
        #initialize
        self.Ki=0.3
        self.Kp=0.3
        self.Kv=0
        self.PrevErrorVel=0.0
        self.CurrentAccel=0.0
        self.CurrentVel=0.0
        self.DesiredVel=desVelInput
        self.ClockStore=self.time.time()
        self.targetAccelOut=0.0
        self.BrakeDecelGain=0.9 # Additional gain for braking deceleration, >1 for harder decel
        

    def updateDesVel(self,updatedVal):
        self.DesiredVel=updatedVal

    def getTargetAccelForll(self):
        return self.targetAccelOut


    def accelCalc(self):
        # r_wh=0.2413 # Radius of Wheel
        # m=1800 # Approx weight of car
        # Calculate desired accel:
        delT=self.time.time()-self.ClockStore
        self.ClockStore=self.time.time()
        errorV=self.DesiredVel-self.CurrentVel
        if delT==0.0: # To avoid div by 0
            errorAccelIMU=0
            # errorAccelDeriv=0
        else:
            errorAccelIMU = (errorV-self.PrevErrorVel)/delT - self.CurrentAccel
        #    errorAccelDeriv=(errorV-PrevErrorVel)/delT
        if abs(errorV)<=1:# Only use PID if errors are small; else use PD
            errorIntegral=0.5*delT*(errorV+self.PrevErrorVel)
        else:
            errorIntegral=0.0
        self.PrevErrorVel=errorV
        self.targetAccelOut=self.Kp*errorV+self.Kv*errorAccelIMU+self.Ki*errorIntegral
        if self.targetAccelOut<0:
            self.targetAccelOut=self.BrakeDecelGain*self.targetAccelOut
        

    def updateCurrVel(self,data):
        if type(data)==type(1.0): # Since this class can be called directly as a script or within ll_controller
            self.CurrentVel=data         
        else:
            self.CurrentVel=data.twist.linear.x
        self.accelCalc()

    def updateCurrAccel(self,data):
        if type(data)==type(1.0):
            self.CurrentAccel=data
        else:
            self.CurrentAccel=data.accel_over_ground
            

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass