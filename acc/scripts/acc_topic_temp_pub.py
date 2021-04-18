#!/usr/bin/env python
import rospy
import numpy as np
import std_msgs.msg
def talker():
    rospy.init_node('enable_publisher', anonymous=True)
    rate = rospy.Rate(10) # 50hz
    accelPub=rospy.Publisher('x_acc/control_input',std_msgs.msg.Float32, queue_size=10)
    while not rospy.is_shutdown():
        inputval=float(input("Please enter the desired accel in m/s^2: "))

        rospy.loginfo('Published Simulated')
        accelPub.publish(inputval)
        #rate.sleep()
if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
#    rate = rospy.Rate(10)
#    hello_str = "hello world"
 #   rospy.loginfo(hello_str)
 #   pub.publish(hello_str)
 #   rospy.spin()
 #   exit(0) 