#!/usr/bin/env python
# Software License Agreement (BSD License)


import rospy
import dbw_mkz_msgs.msg
import std_msgs
import numpy as np
import time
import sys
# Testing Arg Parse
import argparse
#parser=argparse.ArgumentParser()
#parser.add_argument("--thr_cmd", default = 0.2)
#args=parser.parse_args()


global gearSet
gearSet=0
def talker():
        rospy.init_node('ll_controller', anonymous=True)
        rate = rospy.Rate(50) # 50hz
        #throttle_class=dbw_mkz_msgs.msg.ThrottleCmd()
        brake_class=dbw_mkz_msgs.msg.BrakeCmd()
        brake_pub = rospy.Publisher('vehicle/brake_cmd',dbw_mkz_msgs.msg.BrakeCmd, queue_size=1000)
        #throttle_pub=rospy.Publisher('vehicle/throttle_cmd',dbw_mkz_msgs.msg.ThrottleCmd,queue_size=1000)
        # print(gear_class)
        # throttle_class.enable=True
        # throttle_class.pedal_cmd=0.85
        #throttle_class.pedal_cmd=args.thr_cmd
        # throttle_class.pedal_cmd_type=1
        # while not rospy.is_shutdown():
        #         log_Str = (' \n Throttle:', throttle_class.pedal_cmd)
        #         rospy.loginfo(log_Str)
        #         throttle_pub.publish(throttle_class)
        #         throttle_class.enable=True
        # throttle_class.pedal_cmd=0.85
        # BRAKES:
        brake_class.enable=True # Disable Brake, enable throttle
        brake_class.pedal_cmd=1000
        # brake_class.pedal_cmd_type=1 # WORKS,Unitless, range 0.15 to 0.50
        # brake_class.pedal_cmd_type=2 #WORKS# Percent of maximum torque, range 0 to 1
        # brake_class.pedal_cmd_type=3 # WORKSNm, range 0 to 3412, open-loop
        brake_class.pedal_cmd_type=4 # Nm, WORKS range 0 to 3412, closed-loop
        #brake_class.pedal_cmd_type=6 # m/s^2, range 0 to 10
        
        while not rospy.is_shutdown():
                log_Str = (' \n Brake:', brake_class.pedal_cmd,'class:', brake_class.pedal_cmd_type)
                rospy.loginfo(log_Str)
                brake_pub.publish(brake_class)
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
