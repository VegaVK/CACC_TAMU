#!/usr/bin/env python
import rospy
import numpy as np
import std_msgs.msg
def talker():
    rospy.init_node('Sim_publisher', anonymous=True)
    rate = rospy.Rate(50) # 50hz
    tempPub=rospy.Publisher('x_acc/control_input',std_msgs.msg.Float32,queue_size=1000)
    while not rospy.is_shutdown():
                rospy.loginfo('Published Simulated')
                tempAccel=np.sin(0.1*rospy.get_time())
                tempPub.publish(tempAccel)
                print(tempAccel)
                rate.sleep()
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