#!/usr/bin/env python
# Collects current longitdinal acceleration and transmits. Using CAN bus for now; can use filtered IMU as well
import rospy
from dbw_mkz_msgs.msg import BrakeInfoReport
import std_msgs

def talker(data):
    acc_tx_pub = rospy.Publisher('accel_tx/mkz', std_msgs.msg.Float32,queue_size=1000)
    if not rospy.is_shutdown():
        log_Str = ('Published current acceleration over DSRC',data.accel_over_ground)
        rospy.loginfo(log_Str)
        acc_tx_pub.publish(data.accel_over_ground)
def listener():
    # Subscribe to current vehicle acceleration
        rospy.init_node('accel_TX', anonymous=True)
        rospy.Subscriber('vehicle/brake_info_report', BrakeInfoReport,talker)
        rospy.spin()

if __name__=='__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass