#!/usr/bin/env python
# Software License Agreement (BSD License)


import rospy
import dbw_mkz_msgs.msg
from dbw_mkz_msgs.msg import BrakeInfoReport
from dbw_mkz_msgs.msg import BrakeCmd
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
        throttle_class=dbw_mkz_msgs.msg.ThrottleCmd()
        #brake_class=dbw_mkz_msgs.msg.BrakeCmd()
        #brake_pub = rospy.Publisher('vehicle/brake_cmd',dbw_mkz_msgs.msg.BrakeCmd, queue_size=1000)
        throttle_pub=rospy.Publisher('vehicle/throttle_cmd',dbw_mkz_msgs.msg.ThrottleCmd,queue_size=1000)
        #print(gear_class)
        throttle_class.enable=True
        throttle_class.pedal_cmd=0.85
        #throttle_class.pedal_cmd=args.thr_cmd
        throttle_class.pedal_cmd_type=1
        while not rospy.is_shutdown():
                log_Str = (' \n Throttle:', throttle_class.pedal_cmd)
                rospy.loginfo(log_Str)
                throttle_pub.publish(throttle_class)
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
