#!/usr/bin/env python

# ============== Information ==========================================
# Project:      ACC Research
# Description:  Publishing Desired Velocity for ACC
# Date:         July 10 2019
# Filename:     ACC.py
# =====================================================================



import rospy
from std_msgs.msg import Float32
from continental_radar_driver.msg import aheadcar

class ACC:
    def __init__(self):
        self.cd=0 # Current Distance of the Ego car from the Autonomous Car
        self.td=0 # Target Distance of the Ego Car from the Autonomous Car
        self.L = input("Enter Stand Still Distance") # Stand Still Distance
        self.t=input("Enter Time Headway") # Time Headway when the car is Running
        self.relvel=0 # Relative Velocity of the Ego car ahead 
        self.speed=0 # Speed of the Autonomous Car
        self.flag=0
        self.time=0
        self.interr=0


        # Node Initialisation
        rospy.init_node('ACC',anonymous=True)
        # ROS Publishers
        self.velpub=rospy.Publisher('/desired_velocity',Float32,queue_size=1) # Publishing Desired Velocity of the Car to the Controller 
        # ROS Subscribers
        self.AheadCarData=rospy.Subscriber('/AheadCarData',aheadcar,self.ACDCallback)  # Parameters of the Ego Vehicle
        self.speedsub=rospy.Subscriber('/autodrive_sim/output/speed', Float32, self.speedCallback) # Longitudinal Velocity of the Car
 
    
    def ACDCallback(self,msg):
        self.cd=msg.RedDist
        self.relvel=msg.RelVel_Radial
        self.VelocityPub()

    def speedCallback(self, msg):
        self.speed=msg.data

    def VelocityPub(self):
        Kp=10
        Kv=0.5
        self.td=max(self.t*self.speed, self.L)
        e=self.td-self.cd
        if self.flag==0:
            self.time=rospy.get_time()
            self.flag=1
            #V=Kp*e
        else:
            self.interr=self.interr + e*(rospy.get_time()-self.time)
            #V=Kp*e+Kv*self.interr
            Accel=-Kv*(self.relvel) - Kp*(self.cd + self.L + self.t*self.speed)
            V=self.speed+Accel*(rospy.get_time()-self.time)
            self.time=rospy.get_time()
        self.velpub.publish(V)
        rospy.spin()



if __name__=="__main__":
    ACC()
    

'''

So, what's the relative ditance between the cars??
# Let us take that has an input
Is that the only input?? No, but let us proceed with that for now

So, we have the following targets

1. Relative Distance between the car has to be fixed
2. Relative Velocity must be 0 -- This target is automoatically achieved with the first one






'''
