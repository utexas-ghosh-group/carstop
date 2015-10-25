# -*- coding: utf-8 -*-
"""
Created on Fri Sep 25 16:06:55 2015

@author: motro
"""

from timeToAvoid import timeToAvoid
from InitializeParams import InitializeParams
import numpy as np
#import time


initializationType = 'Rural Road'

numTrials = 1000

outputFile = 'testSimulation.csv'


output = []

# run simulation
#starttime = time.clock()
for trial in range(numTrials):
    params = InitializeParams(initializationType)
    trialOut = timeToAvoid(*params.outputList())
    if trialOut is None:
        print params.outputList()
    output.append( trialOut )
#looptime = time.clock()

# output to .csv
output = np.array(output)
#arraytime = time.clock()

header = ('ego_speed,ego_acceleration,ego_angular_velocity,' +
        'collision_angle,' +
        'alter_speed,alter_acceleration,alter_angular_velocity,' +
        'time_to_avoid')
np.savetxt(outputFile,output,'%.3f',',','\n',header)

#writetime = time.clock()

#print 'loop: ',looptime-starttime
#print 'array: ',arraytime-looptime # loop takes vast majority
#print 'write: ',writetime-arraytime