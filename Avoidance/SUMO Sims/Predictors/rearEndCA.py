# -*- coding: utf-8 -*-
"""
designed to predict a rearEndBrake controlled vehicle
1/20/16
"""
import numpy as np
import pandas as pd   
from usefulFunctions import CA_physics 

def movePhysics(vstate,v,a,predictTime):
    newstate = vstate.copy()
    dT = predictTime - vstate.time
    newx, newv = CA_physics(dT, vstate.x, v, a, cutoff=3.5)
    newstate.x = newx
    newstate.speed = newv
    newstate.y = -8.25
    newstate.time = predictTime
    return newstate


class RearEndCA:
    def __init__(self, trueTrajectory, dt=.1):
        self.dt = dt
        
    def predict(self, vData, predictTimes):
        nobs = vData.shape[0]
        speed = np.array(vData.loc[:,'speed'])
        filterLen = min(nobs,8)

        if nobs == 1:
            chosenAccel = 0
        else:
            chosenAccel = np.mean(np.diff(speed[-filterLen:]))
        if nobs < 10: # early on, don't want to guess high accel
            chosenAccel = chosenAccel * (nobs/10)**2
            
        # find current speed
        filteredSpeed = speed[len(speed)-filterLen:]
        for time in range(filterLen):
            filteredSpeed[time] += chosenAccel*(filterLen-1-time)
        chosenSpeed = np.mean(filteredSpeed)
        chosenAccel = chosenAccel / self.dt
        
        # now you can predict
        # return a dataframe of answers
        vcurrentState = vData.iloc[vData.shape[0]-1]
        returnedStates = vData[vData['time']<0] # empty
        for time in predictTimes:
            vNextState = movePhysics(vcurrentState,chosenSpeed,chosenAccel,time)
            returnedStates = returnedStates.append(vNextState)
        return returnedStates