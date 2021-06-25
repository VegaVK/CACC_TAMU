#!/usr/bin/env python
# Software License Agreement (BSD License)


import rospy
import dbw_mkz_msgs.msg
import std_msgs
import numpy as np
import time
import math
import sys
def talker():
    rospy.init_node('steering_sweeper', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    steer_class=dbw_mkz_msgs.msg.SteeringCmd()
    steer_pub = rospy.Publisher('vehicle/steering_cmd',dbw_mkz_msgs.msg.SteeringCmd, queue_size=2)
    inputval=float(input("Please enter the desired steering angle in DEGREES: "))
    desSteerAngleRad=inputval*(180/math.pi) # Need to convert from deg to radians for Steering Cmd
    steer_class.enable=True
    steer_class.steering_wheel_angle_cmd=desSteerAngleRad
    steer_class.steering_wheel_angle_velocity=1 # Hard coded here.
    while not rospy.is_shutdown():
        rospy.loginfo_once('Published Steering Command')
        steer_pub.publish(steer_class)
        rate.sleep()
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        print("Shutting down")

