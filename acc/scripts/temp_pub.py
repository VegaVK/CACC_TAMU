#!/usr/bin/env python
import rospy
import numpy as np
import std_msgs.msg
def talker():
    rospy.init_node('enable_publisher', anonymous=True)
    rate = rospy.Rate(50) # 50hz
    tempPub=rospy.Publisher('vehicle/dbw_enabled',std_msgs.msg.Bool,queue_size=1000)
    while not rospy.is_shutdown():
                rospy.loginfo('Published Simulated')
                tempPub.publish(True)
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