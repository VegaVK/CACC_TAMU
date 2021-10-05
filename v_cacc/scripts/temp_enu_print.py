#!/usr/bin/env python3
# prints ENU coords, temp for testing
import rospy
import std_msgs
from sensor_msgs.msg import Image, NavSatFix

import pymap3d as pm
import numpy as np



def main():
    rospy.init_node('enuPrinter', anonymous=True)
    printer=enuPrinter()
    rospy.spin()


class enuPrinter():
    def __init__(self):

        ## **** GPS ORIGIN ****
        self.lat0 = 30.632913
        self.lon0 = -96.481894 # deg
        self.h0 = 54.3     # meters
        rospy.Subscriber('/piksi/navsatfix_best_fix', NavSatFix,self.enuConverter)

    def enuConverter(self,data):
        tempVal=pm.geodetic2enu(data.latitude,data.longitude,data.altitude, self.lat0, self.lon0, self.h0)
        print(tempVal)
   
if __name__=='__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("Shutting down")