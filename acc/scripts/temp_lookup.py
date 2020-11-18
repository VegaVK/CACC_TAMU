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

CURRENTVEL=6
targetAccel=0.6

Mapdata=sio.loadmat('LookupPy_EngineMKZ.mat')
CMDARR_THR=np.array(Mapdata['X_Lup']).T
VELGRID_THR=np.array(Mapdata['Y_Lup']).T
z1=Mapdata['Z_Lup']
Mapdata=sio.loadmat('LookupPy_BrakeMKZ.mat')
CMDARR_BRK=np.array(Mapdata['X_LupBr']).T
VELGRID_BRK=np.array(Mapdata['Y_LupBr']).T
z2=Mapdata['Z_LupBr']
interpFun2DEng=interpolate.interp2d(CMDARR_THR,VELGRID_THR,z1)
interpFun2DBrk=interpolate.interp2d(CMDARR_BRK,VELGRID_BRK,z2)
ZnewEng=interpFun2DEng(CMDARR_THR[0],CURRENTVEL) # Bunch of Accelerations
ZnewBrk=interpFun2DBrk(CMDARR_BRK[0],CURRENTVEL) # Bunch of Accelerations

# print(ZnewBrk)
# print(thrGrid)
# print(velGrid)
if (targetAccel<=0):
    throttle_out=0.0
    if (targetAccel<=-4):
        brake_out=3412
    else: #Quite convoluted, see if there's a cleaner way later
        for idx in range(0,(ZnewBrk.shape[0]-2)):
            # print(idx)
            lower=ZnewBrk[idx]
            upper=ZnewBrk[idx+1]
            if (targetAccel<=lower)&(targetAccel>upper): #Reverse, since negative values for break
                # print('satisfied')
                brake_out=(0.05)*(targetAccel-lower)/(upper-lower)+CMDARR_BRK[0][idx]     
                break
            else:
                idx=idx+1
        
else:
    brake_out=0
    if(targetAccel>3):
        throttle_out=0.8
    else: #Quite convoluted, see if there's a cleaner way later
        for idx in range(0,(ZnewEng.shape[0]-2)):
            lower=ZnewEng[idx]
            upper=ZnewEng[idx+1]
            if (targetAccel>=lower)&(targetAccel<upper):
                throttle_out=(0.05)*(targetAccel-lower)/(upper-lower)+CMDARR_THR[0][idx]     
            else:
                idx=idx+1



print('throttleOut: '+str(throttle_out))
print('brakeOut: ' +str(brake_out))

''' #print(scipy.interp.bisplrep(temp[:][0],temp[:][1],temp[:][2]))
fig=plt.figure()
ax=fig.add_subplot(111,projection='3d')
ax.scatter(temp[:,0],temp[:,1],temp[:,2])
plt.show()
print(np.amax(temp[:,3]))
print(temp.shape)
#print(temp[:,0],temp[:,1],temp[:,2]) '''
