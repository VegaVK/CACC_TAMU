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
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
import dbw_mkz_msgs.msg
from dbw_mkz_msgs.msg import BrakeInfoReport
import std_msgs
import numpy as np
import time
global gearSet
gearSet=0
def callback(data):
        global gearSet
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
        if data.data<0:
                throttle_class.enable=False 
                throttle_class.pedal_cmd=0
                throttle_class.pedal_cmd_type=0
                brake_class.enable=True# Enable Brake, disable throttle
                brake_class.pedal_cmd_type= 2# Directly supply deceleration value
                brake_class.pedal_cmd=6*abs(data.data) # Only accepts positive values
        else:
                brake_class.enable=False # Disable Brake, enable throttle
                brake_class.pedal_cmd=0
                brake_class.pedal_cmd_type=0
                throttle_class.enable=True
                throttle_class.pedal_cmd_type=1 # Using 0.15 to 0.8
                throttle_class.pedal_cmd=0.15+0.19667*data.data
        if not rospy.is_shutdown():
                log_Str = ('\n Brake: ', brake_class.pedal_cmd, ' \n Throttle:', throttle_class.pedal_cmd)
                rospy.loginfo(log_Str)
                brake_pub.publish(brake_class)
                throttle_pub.publish(throttle_class)

def listener():


    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    
    # Temp Publisher to test code internally
        rospy.init_node('ll_controller', anonymous=True)
        rate = rospy.Rate(50) # 50hz
        rospy.Subscriber('x_acc/control_input', std_msgs.msg.Float32, callback)
        rospy.loginfo(rospy.get_caller_id()+'x_acc received')
        rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass