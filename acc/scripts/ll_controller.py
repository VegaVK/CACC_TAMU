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



def pedalPublisher(data):
    global interpFun2DEng
    global interpFun2DBrk
    global CURRENTVEL
    global CMDARR_THR
    global VELGRID_THR
    global CMDARR_BRK
    global VELGRID_BRK
    throttle_class=dbw_mkz_msgs.msg.ThrottleCmd()
    brake_class=dbw_mkz_msgs.msg.BrakeCmd()
    brake_pub = rospy.Publisher('vehicle/brake_cmd',dbw_mkz_msgs.msg.BrakeCmd, queue_size=10)
    throttle_pub=rospy.Publisher('vehicle/throttle_cmd',dbw_mkz_msgs.msg.ThrottleCmd,queue_size=10)
    r_wh=0.2413 # Radius of Wheel
    m=1800 # Approx weight of car
    # Calculate Throttle Command first:
    targetAccel=data.data # From ACC Controller
    ZnewEng=interpFun2DEng(CMDARR_THR[0],CURRENTVEL) # Bunch of Accelerations
    ZnewBrk=interpFun2DBrk(CMDARR_BRK[0],CURRENTVEL) # Bunch of Accelerations
    if (targetAccel<=0):
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
                    brake_out=(0.05)*(targetAccel-lower)/(upper-lower)+CMDARR_BRK[0][idx]     
                    break
                else:
                    # idx=idx+1
                    if idx >=(ZnewBrk.shape[0]-2):
                        brake_out=3400 # TODO: Double check this, was modified on Jun17, was throttle_out=0.8 earlier.
                    else:
                        continue

                    
            
    else:
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
                print(lower)
                upper=ZnewEng[idx+1]
                if  (targetAccel<upper):
                    throttle_out=(0.05)*(upper-targetAccel)/(upper-lower)+CMDARR_THR[0][idx]   #  Seems wrong, temp fix below
                    # throttle_out= 
                    throttle_out=CMDARR_THR[0][idx] 
                else:
                    # idx=idx+1
                    if (idx >=(ZnewEng.shape[0]-2)) and targetAccel>=upper:
                        throttle_out=0.8 
                    elif targetAccel<=upper: # REALLY  BAD FIX< IN RUNWAY< REMOVE
                        throttle_out=0.5
                    else:
                        continue
                    # pass
    if targetAccel<0:
            throttle_class.enable=False 
            throttle_class.pedal_cmd=0
            throttle_class.pedal_cmd_type=0
            brake_class.enable=True# Enable Brake, disable throttle
            #brake_class.pedal_cmd_type= 2# Mode2, Percent of maximum torque, from 0 to 1
            brake_class.pedal_cmd_type= 4# Mode1, Unitless, Range 0.15 to 0.5
            #brake_class.pedal_cmd=brakeGain*abs(data.data)*m*r_wh/ # For Mode 2
            brake_class.pedal_cmd=brake_out # For Mode 1
    else:
            brake_class.enable=False # Disable Brake, enable throttle
            brake_class.pedal_cmd=0
            brake_class.pedal_cmd_type=0
            throttle_class.enable=True
            throttle_class.pedal_cmd_type=1 # Using 0.15 to 0.8
            throttle_class.pedal_cmd=throttle_out# Originally stable, remove after enginemap works
            #throttle_class.pedal_cmd=0.6*throttle_out
    if not rospy.is_shutdown():
            log_Str = ('\n Brake: ', brake_class.pedal_cmd, ' \n Throttle:', throttle_class.pedal_cmd)
            rospy.loginfo(log_Str)
            brake_pub.publish(brake_class)
            throttle_pub.publish(throttle_class)

def velfun(data):
    global CURRENTVEL
    CURRENTVEL=data.twist.linear.x

def listener():
    # Subscribe to external ACC/CACC controller
    rospy.init_node('ll_controller', anonymous=True)
    rate = rospy.Rate(20) # 50hz
    # rospy.Subscriber('vehicle/steering_report',dbw_mkz_msgs.msg.SteeringReport,velfun)
    rospy.Subscriber('/vehicle/twist', TwistStamped,velfun)
    rospy.Subscriber('/x_acc/control_input', std_msgs.msg.Float32, pedalPublisher)
    rospy.loginfo_once(rospy.get_caller_id()+'x_acc received')
    rospy.spin()

if __name__=='__main__':
    try:
        global interpFun2DEng
        global interpFun2DBrk
        global CMDARR_THR
        global VELGRID_THR
        global CMDARR_BRK
        global VELGRID_BRK

        # global interpFun2DEng
        # global interpFun2DBrk
        global CURRENTVEL
        # global CMDARR_THR
        # global VELGRID_THR
        # global CMDARR_BRK
        # global VELGRID_BRK
        CURRENTVEL=0.0
        Mapdata=sio.loadmat('LookupPy_EngineMKZ.mat')
        CMDARR_THR=np.array(Mapdata['X_Lup']).T
        VELGRID_THR=np.array(Mapdata['Y_Lup']).T
        z1=Mapdata['Z_Lup']
        Mapdata=sio.loadmat('LookupPy_BrakeMKZ.mat')
        CMDARR_BRK=np.array(Mapdata['X_LupBr']).T
        VELGRID_BRK=np.array(Mapdata['Y_LupBr']).T
        z2=Mapdata['Z_LupBr']
        interpFun2DEng=interpolate.interp2d(CMDARR_THR,VELGRID_THR,z1)
        interpFun2DBrk=interpolate.interp2d(CMDARR_BRK,VELGRID_BRK,z2)
        listener()
    except rospy.ROSInterruptException:
        pass
