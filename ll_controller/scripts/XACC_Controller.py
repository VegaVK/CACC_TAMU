#!/usr/bin/env python

# ============== Information ==========================================
# Project:      ACC Research
# Description:  Publishing Desired Torques for XACC into CAN
# Car:          Autodrive - Chevrolet Bolt
# Date:         July 10 2019
# Filename:     XACC_Controller.py
# =====================================================================

import rospy
from std_msgs.msg import Float32, Int64


class Controller:
    def __init__(self):

        #Subscriber
        self.accel_sub=rospy.Subscriber('/XACC/Acceleration',Float32, self.callback)

        #Publishers
        self.throttle_pub=rospy.Publisher('can_throttle_auto',Int64,queue_size=1)
        self.brake_pub=rospy.Publisher('can_brake_auto',Int64,queue_size=1)

        #Acceleration of Car
        self.Accel=0

        #Tau of Filter
        self.tau=0.2

        #Throttle Output Value
        self.throttle=0

        #Brake Output Value
        self.brake=0

        # Previous Instant Acceleration for Filtering
        self.prevAccel=0

        # To find First Instance
        self.flag=0

        # Car Parameters
        self.mass=1950
        self.TireRadius=0.31


    def callback(self,msg):
        self.Accel=msg.data
        self.filtering()
        
    # Filters the Acceleration Value - The Value of acceleration is limited while publishing the Throttle and Brake Torques
    def filtering(self):
        FinalAcc=0
        if self.flag==0:
            self.flag=1
            FinalAcc=self.Accel
            self.prevAccel=self.Accel
            self.publish(FinalAcc)
        else:
            FinalAcc=self.prevAccel*self.tau + self.Accel*(1-self.tau)
            self.prevAccel=FinalAcc
            self.publish(FinalAcc)

    #Publishing the torque and Brake Values
    def publish(self,msg):
        FinalAcc=msg.data
        if FinalAcc>=0:
            self.throttle=Int64(data=FinalAcc*self.mass*self.TireRadius)
            self.throttle=min(self.throttle,1199) # Max Throttle Torque Value is 1199.5
            self.brake=Int64(data=0)
        else:
            self.brake=Int64(data=FinalAcc*self.mass*self.TireRadius)
            self.brake=max(self.throttle,-848) # Max Brake Torque is -848
            self.throttle=Int64(data=0)
        self.throttle_pub.publish(self.throttle)
        self.brake_pub.publish(self.brake)




if __name__=="__main__":
    rospy.init_node('XACC_Controller',anonymous=True)
    Controller()
    rospy.spin()