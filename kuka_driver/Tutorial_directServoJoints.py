# -*- coding: utf-8 -*-
"""
About the script:
An exmple on controlling KUKA iiwa robot from
Python3 using the iiwaPy3 class

Modified on 3rd-Jan-2021

@author: Mohammad SAFEEA

"""
import math
import numpy as np
import time
from datetime import datetime

from iiwaPy3 import iiwaPy3

start_time = datetime.now()


# returns the elapsed seconds since the start of the program
def getSecs():
    dt = datetime.now() - start_time
    secs = (dt.days * 24 * 60 * 60 + dt.seconds) + dt.microseconds / 1000000.0
    return secs


ip = '172.31.1.147'
# ip='localhost'
iiwa = iiwaPy3(ip)
iiwa.setBlueOn()
time.sleep(2)
iiwa.setBlueOff()
# read some data from the robot
try:

    # Move to an initial position    
    initPos = [-np.pi/6, np.pi/6, 0.0, -np.pi/2,0,np.pi/3,np.pi/2-np.pi/6-np.pi];
    initVel = [0.1]
    iiwa.movePTPJointSpace(initPos, initVel)

    ref_traj = np.load("traj2.npy")

    counter = 0
    index = 0

    jpos = iiwa.getJointsPos()
    jpos0_6 = jpos[index]
    #iiwa.realTime_startDirectServoJoints()

    t0 = getSecs()
    t_0 = getSecs()
    while counter < len(ref_traj):
        iiwa.movePTPJointSpace(ref_traj[counter].tolist(), [0.2])
        counter = counter + 1
        print("Here")

    deltat = getSecs() - t0;
    #iiwa.realTime_stopDirectServoJoints()

    # Move to an initial position    
    jPos = [-np.pi/6, np.pi/6, 0.0, -np.pi/2,0,np.pi/3,np.pi/2-np.pi/6-np.pi];
    vRel = [0.1]
    iiwa.movePTPJointSpace(jPos, vRel)
    # Print some statistics
    print('update freq')
    print(counter / deltat)
except:
    print('an error happened')
# Close connection    
iiwa.close()
