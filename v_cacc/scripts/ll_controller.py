#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


import rospy
import dbw_mkz_msgs.msg
from dbw_mkz_msgs.msg import BrakeInfoReport
import std_msgs
import numpy as np
import time
import scipy.io as sio
from scipy import interpolate

global gearSet
global THROTTLE_OUT
THROTTLE_OUT=0.0
gearSet=0
class Vehicle:
        Vel=dbw_mkz_msgs.msg.SteeringReport()
def callback(data):
        global gearSet
        global interpFun2D
        global THROTTLE_OUT
        print('start callbak')
        throttle_class=dbw_mkz_msgs.msg.ThrottleCmd()
        brake_class=dbw_mkz_msgs.msg.BrakeCmd()
        brake_pub = rospy.Publisher('vehicle/brake_cmd',dbw_mkz_msgs.msg.BrakeCmd, queue_size=1000)
        throttle_pub=rospy.Publisher('vehicle/throttle_cmd',dbw_mkz_msgs.msg.ThrottleCmd,queue_size=1000)
        #### Only use if Need to switch Gear in gazebo simulation.
        # #if gearSet==0:
        # gear_pub=rospy.Publisher('vehicle/gear_cmd',dbw_mkz_msgs.msg.GearCmd,queue_size=1000)
        # gear_class=dbw_mkz_msgs.msg.GearCmd()
        # gear_cmd_drive_class=dbw_mkz_msgs.msg.Gear()
        # gear_cmd_drive_class.gear=4
        # gear_class.cmd=gear_cmd_drive_class
        # rospy.loginfo('Gear Published to Drive')
        # print(gearSet)
        # # PRessing brake before shifting out of gear
        # brake_class.enable=True# Enable Brake, disable throttle
        # brake_class.pedal_cmd_type=6 # Decel Val
        # brake_class.pedal_cmd=0 # Only accepts positive values
        # brake_pub.publish(brake_class)
        # gear_pub.publish(gear_class)
        # gearSet=4
        # print(gear_class)
        r_wh=0.2413 # Radius of Wheel
        m=1800 # Approx weight of car
        brakeGain=6 # Unused
        # Calculate Throttle Command first:
        
        CURRENTVEL=Vehicle.Vel.speed
        TARGETACCEL=data.data # From ACC Controller
        thr_temp=np.linspace(0.1,0.9,17)
        vel_temp=CURRENTVEL
        Znew=interpFun2D(thr_temp,vel_temp) # Bunch of Accelerations
        if (TARGETACCEL<=0):
                THROTTLE_OUT=0.0
        elif(TARGETACCEL>3):
                THROTTLE_OUT=0.8
        else: #Quite convoluted, see if there's a cleaner way later
                for idx in range(0,(Znew.shape[0]-1)):
                        lower=Znew[idx]
                        upper=Znew[idx+1]
                        if (TARGETACCEL>=lower)&(TARGETACCEL<upper):
                                THROTTLE_OUT=0.05*(TARGETACCEL-lower)/(upper-lower)+thr_temp[idx]     
                        else:
                                idx=idx+1
        if data.data<0:
                throttle_class.enable=False 
                throttle_class.pedal_cmd=0
                throttle_class.pedal_cmd_type=0
                brake_class.enable=True# Enable Brake, disable throttle
                #brake_class.pedal_cmd_type= 2# Mode 2, Percent of maximum torque, from 0 to 1
                brake_class.pedal_cmd_type= 1# Mode 1, Unitless, Range 0.15 to 0.5
                #brake_class.pedal_cmd=brakeGain*abs(data.data)*m*r_wh/ # For Mode 2
                brake_class.pedal_cmd=0.3*abs(data.data)+0.15 # For Mode 1
        else:
                brake_class.enable=False # Disable Brake, enable throttle
                brake_class.pedal_cmd=0
                brake_class.pedal_cmd_type=0
                throttle_class.enable=True
                throttle_class.pedal_cmd_type=1 # Using 0.15 to 0.8
                #throttle_class.pedal_cmd=0.15+0.19667*data.data# Originally stable, remove after enginemap works
                throttle_class.pedal_cmd=THROTTLE_OUT

        if not rospy.is_shutdown():
                log_Str = ('\n Brake: ', brake_class.pedal_cmd, ' \n Throttle:', throttle_class.pedal_cmd)
                rospy.loginfo(log_Str)
                brake_pub.publish(brake_class)
                throttle_pub.publish(throttle_class)

def velfun(data):
    Vehicle.Vel.speed=data.speed
def listener():
    # Subscribe to external ACC/CACC controller
        rospy.init_node('ll_controller', anonymous=True)
        rate = rospy.Rate(50) # 50hz
        rospy.Subscriber('vehicle/steering_report',dbw_mkz_msgs.msg.SteeringReport,velfun)
        rospy.Subscriber('x_acc/control_input', std_msgs.msg.Float32, callback)
        rospy.loginfo(rospy.get_caller_id()+'x_acc received')

        rospy.spin()

if __name__=='__main__':
    try:
        Mapdata=sio.loadmat('LookupTable.mat')
        global interpFun2D
        thrGrid=Mapdata['X_Lup']
        velGrid=Mapdata['Y_Lup']
        z1=Mapdata['Z_Lup']
        interpFun2D=interpolate.interp2d(thrGrid,velGrid,z1)
        listener()
    except rospy.ROSInterruptException:
        pass