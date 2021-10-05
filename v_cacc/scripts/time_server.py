#!/usr/bin/env python
# Publishes a 'sim' time for reading from bag file data.
import rospy
import std_msgs.msg
import numpy as np
rospy.init_node('time_server', anonymous=True)
timSrvPub = rospy.Publisher('time_topic', std_msgs.msg.Time, queue_size=1)
startTime=rospy.Time.now()

r = rospy.Rate(20) # 10hz
timeClass=std_msgs.msg.Time()
rospy.loginfo_once('Started publishing time topic...')
while not rospy.is_shutdown():
    currentTime=rospy.Time.now()
    diff=rospy.Time.from_sec(currentTime.to_sec()-startTime.to_sec())
    # print(type(diff))
    timSrvPub.publish(diff)
    r.sleep()