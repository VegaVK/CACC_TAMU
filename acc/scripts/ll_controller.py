#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.

import rospy
import dbw_mkz_msgs.msg
from geometry_msgs.msg import TwistStamped
from rospy.core import logwarn
import std_msgs
import numpy as np
import time
import scipy.io as sio
from scipy import interpolate
from velTracker import velTrackClass

# TODO: if accel recieved is zero, maintain current velocity at time of command
# TODO: remove self.variables, switch to class based model

def main():
    
    ll_contr_inst=llContClass()
    rospy.spin()
   
class llContClass():
    def __init__(self):
        self.CurrentVel=0.0 # in m/s
        self.CurrentAccel=0.0
        self.Mapdata=sio.loadmat('LookupPy_EngineMKZ.mat')
        self.CmdArrThr=np.array(self.Mapdata['X_Lup']).T
        self.VelGridThr=np.array(self.Mapdata['Y_Lup']).T
        self.z1=self.Mapdata['Z_Lup']
        self.Mapdata=sio.loadmat('LookupPy_BrakeMKZ.mat')
        self.CmdArrBrk=np.array(self.Mapdata['X_LupBr']).T
        self.VelGridBrk=np.array(self.Mapdata['Y_LupBr']).T
        self.z2=self.Mapdata['Z_LupBr']
        self.interpFun2DEng=interpolate.interp2d(self.CmdArrThr,self.VelGridThr,self.z1)
        self.interpFun2DBrk=interpolate.interp2d(self.CmdArrBrk,self.VelGridBrk,self.z2)
        self.throttle_class=dbw_mkz_msgs.msg.ThrottleCmd()
        self.brake_class=dbw_mkz_msgs.msg.BrakeCmd()
        rospy.init_node('ll_controller', anonymous=True)
        self.velTrackerInst=velTrackClass(0.0)
        self.brake_pub = rospy.Publisher('vehicle/brake_cmd',dbw_mkz_msgs.msg.BrakeCmd, queue_size=2)
        self.throttle_pub=rospy.Publisher('vehicle/throttle_cmd',dbw_mkz_msgs.msg.ThrottleCmd,queue_size=2)
        # Subscribe to external ACC/CACC controller
        # rospy.Subscriber('vehicle/steering_report',dbw_mkz_msgs.msg.SteeringReport,velfun)
        rospy.Subscriber('/vehicle/twist', TwistStamped,self.velfun)
        rospy.Subscriber('/x_acc/control_input', std_msgs.msg.Float32, self.pedalPublisher)
        rospy.Subscriber('/vehicle/brake_info_report', dbw_mkz_msgs.msg.BrakeInfoReport,self.accelfun)
        rospy.loginfo_once(rospy.get_caller_id()+'ll_controller Started')


    def pedalPublisher(self,data):
        
        # r_wh=0.2413 # Radius of Wheel
        # m=1800 # Approx weight of car
        if data.data==0.0: #command from ACC is zero, so maintain current velocity
            self.velTrackerInst.updateDesVel(self.CurrentVel) # supplies current velocity
            self.velTrackerInst.updateCurrAccel(self.CurrentAccel)
            self.velTrackerInst.updateCurrVel(self.CurrentVel)
            targetAccel=self.velTrackerInst.getTargetAccelForll()
        else:
            targetAccel=data.data # From ACC Controller
        # Calculate Throttle Command first:
        ZnewEng=self.interpFun2DEng(self.CmdArrThr[0],self.CurrentVel) # Bunch of Accelerations
        ZnewBrk=self.interpFun2DBrk(self.CmdArrBrk[0],self.CurrentVel) # Bunch of Accelerations
        if (targetAccel<0):
            throttle_out=0.0
            if (targetAccel<=-4):
                brake_out=3412
            else: #Quite convoluted, see if there's a cleaner way later
                for idx in range(0,(ZnewBrk.shape[0]-1)):
                    # print(idx)
                    # print(ZnewBrk.shape[0]-1)
                    if idx==0:
                        lower=0 # TODO: First element, force it to be zero (possibly edit this in the map directly in future)
                    else:
                        lower=ZnewBrk[idx]
                    upper=ZnewBrk[idx+1]
                    if (targetAccel<=lower) and (targetAccel>upper): #Reverse, since negative values for break
                        # print('satisfied')
                        brake_out=(100)*(targetAccel-lower)/(upper-lower)+self.CmdArrBrk[0][idx]     
                        break
                    else:
                        # idx=idx+1
                        if idx >=(ZnewBrk.shape[0]-2):
                            brake_out=3400 # TODO: Double check this, was modified on Jun17, was throttle_out=0.8 earlier.
                        else:
                            continue

                        
                
        elif(targetAccel>=0):
            brake_out=0
            if(targetAccel>3):
                throttle_out=0.8
            else: #Quite convoluted, see if there's a cleaner way later
                # print(ZnewEng)
                # print(ZnewEng.shape[0]-1)
                for idx in range(0,(ZnewEng.shape[0]-1)):
                    # print('idx:')
                    # print(idx)
                    if idx==0:
                        lower=0 # TODO: First element, force it to be zero (possibly edit this in the map directly in future)
                    else:
                        lower=ZnewEng[idx]
                    
                    
                    upper=ZnewEng[idx+1]
                    # print(upper)
                    if  (targetAccel<upper):
                        throttle_out=self.CmdArrThr[0][idx+1]-(0.05)*(upper-targetAccel)/(upper-lower)   
                        break

                    else:
                        # idx=idx+1
                        if (idx >=(ZnewEng.shape[0]-2)) and targetAccel>=upper:
                            throttle_out=0.8 

                        else:
                            continue
        if targetAccel<0:
                self.throttle_class.enable=False 
                self.throttle_class.pedal_cmd=0
                self.throttle_class.pedal_cmd_type=0
                self.brake_class.enable=True# Enable Brake, disable throttle
                #brake_class.pedal_cmd_type= 2# Mode2, Percent of maximum torque, from 0 to 1
                self.brake_class.pedal_cmd_type= 4# Mode1, Unitless, Range 0.15 to 0.5
                #brake_class.pedal_cmd=brakeGain*abs(data.data)*m*r_wh/ # For Mode 2
                self.brake_class.pedal_cmd=brake_out # For Mode 1
        else:
                self.brake_class.enable=False # Disable Brake, enable throttle
                self.brake_class.pedal_cmd=0
                self.brake_class.pedal_cmd_type=0
                self.throttle_class.enable=True
                self.throttle_class.pedal_cmd_type=1 # Using 0.15 to 0.8
                self.throttle_class.pedal_cmd=throttle_out# Originally stable, remove after enginemap works
                #throttle_class.pedal_cmd=0.6*throttle_out
        if not rospy.is_shutdown():
                log_Str = ('\n Brake: ', self.brake_class.pedal_cmd, ' \n Throttle:', self.throttle_class.pedal_cmd)
                rospy.loginfo(log_Str)
                self.brake_pub.publish(self.brake_class)
                self.throttle_pub.publish(self.throttle_class)

    def velfun(self,data):
        self.CurrentVel=data.twist.linear.x
    def accelfun(self,data):
        self.CurrentAccel=data.accel_over_ground

if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")