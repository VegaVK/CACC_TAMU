#!/usr/bin/env python
# Software License Agreement (BSD License)
#

## open saved experiment data, fit a surface and save fit.

import rospy
import dbw_mkz_msgs.msg 
import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import scipy.io as sio
from scipy import interpolate


CURRENTVEL=10
TARGETACCEL=1
data=sio.loadmat('LookupTable.mat')
thrGrid=data['X_Lup']
velGrid=data['Y_Lup']
z1=data['Z_Lup']
interpFun2D=interpolate.interp2d(thrGrid,velGrid,z1)

thr_temp=np.linspace(0.1,0.9,17)
vel_temp=CURRENTVEL
Znew=interpFun2D(thr_temp,vel_temp) # Bunch of Accelerations
if (TARGETACCEL<0):
    THROTTLE_OUT=0
elif(TARGETACCEL>3.8):
    THROTTLE_OUT=0.8
else: #Quite convoluted, see if there's a cleaner way later
    for idx in range(0,(Znew.shape[0]-2)):
        lower=Znew[idx]
        upper=Znew[idx+1]
        if (TARGETACCEL>=lower)&(TARGETACCEL<upper):
            TARGETACCEL=0.05*(TARGETACCEL-lower)/(upper-lower)+thr_temp[idx]     
        else:
            idx=idx+1
        

print(TARGETACCEL)

''' #print(scipy.interp.bisplrep(temp[:][0],temp[:][1],temp[:][2]))
fig=plt.figure()
ax=fig.add_subplot(111,projection='3d')
ax.scatter(temp[:,0],temp[:,1],temp[:,2])
plt.show()
print(np.amax(temp[:,3]))
print(temp.shape)
#print(temp[:,0],temp[:,1],temp[:,2]) '''
