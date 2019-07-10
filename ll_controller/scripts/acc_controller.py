#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
## Standard ACC implementation, with no use of acceleration from radar, or otherwise

import rospy
import std_msgs
import dbw_mkz_msgs
import delphi_esr_msgs

global CipvID # Closest in path vehicle's ID
global Kp # Controller gains
global Kv 
global L # Stand Still Distance
global CurrentVel # Velocity of Ego vehicle
global timeHeadway # Desired Time Headway
CurrentVel=0 # Start condition
timeHeadway=0.5 # In seconds
L=20# in meters
Kp=0.01
Kv=0.01
CipvID=0
# Callback1 uses the identified track and current velocity to calculate controller effort(accel) and publishes it
def callback1(data):
    global CipvID
    global Kp
    global Kv
    global CurrentVel
    global timeHeadway
    acc_class=std_msgs.msg.Float32()
    acc_pub = rospy.Publisher('x_acc/control_input',std_msgs.msg.Float32,queue_size=1000)
    if  data.track_ID==CipvID:
        print(CipvID)
        meas_range=data.track_range # x_{i-1} -x_i
        meas_range_rate=data.track_range_rate
        acc_class.data=-Kp*(-meas_range+L+CurrentVel*timeHeadway)-Kv*(-meas_range_rate)
        if not rospy.is_shutdown():
                    log_Str = ('Published target Controller Output ( Pure ACC):',acc_class)
                    rospy.loginfo(log_Str)
                    acc_pub.publish(acc_class)
    elif CipvID==0:
        acc_class.data=0 # No controller output if not found
        # rospy.logwarn('No ACC Target found')
        # if not rospy.is_shutdown():
        #             log_Str = ('Published target Controller Output ( Pure ACC):',acc_class)
        #             rospy.loginfo(log_Str)
        #             acc_pub.publish(acc_class)
# Callback2 Identifies the relevant track for ACC
def callback2(data2):
    from dbw_mkz_msgs.msg import SteeringReport
    from delphi_esr_msgs.msg import EsrStatus4
    global CipvID
    print(data2.pathIdAcc)
    if (not (data2.pathIdAcc==0) or not(data2.pathIdAccStat==0)): # pick one or the other
        
        if data2.pathIdAcc==0:
            CipvID=data2.pathIdAccStat-1
        else:
            CipvID=data2.pathIdAcc-1
    else:
        CipvID=0 # Or else, No track found
# Callback 3 is for obtaining ego vehicle velocity
def callback3(data3):
    global CurrentVel
    CurrentVel=data3.speed
    # print (CurrentVel)
def listener():
        from dbw_mkz_msgs.msg import SteeringReport
        from delphi_esr_msgs.msg import EsrStatus4
        rospy.init_node('acc_controller', anonymous=True)
        rate = rospy.Rate(50) # 50hz
        rospy.Subscriber('vehicle/steering_report', dbw_mkz_msgs.msg.SteeringReport, callback3)
        rospy.Subscriber('parsed_tx/radarstatus4', delphi_esr_msgs.msg.EsrStatus4, callback2)
        rospy.Subscriber('parsed_tx/radartrack', delphi_esr_msgs.msg.EsrTrack,  callback1)
        rospy.loginfo(rospy.get_caller_id()+'x_acc received')
        rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
