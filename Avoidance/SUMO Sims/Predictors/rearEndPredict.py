# -*- coding: utf-8 -*-
"""
designed to predict a rearEndBrake controlled vehicle
11/10/15
"""
import numpy as np
import pandas as pd
    
# given model that car has 0 accel before brakeTime and constant accel after,
# find accel that fits the best and its likelihood
def brakeLikelihood(speed, accel, brakeTime):
    sensorNoiseSD = .25/np.sqrt(2) # knowledge of DSRC
    brakeLikelihood = np.log(1-1/100)
    # knowledge of simulator, should have error added to it at some point        
    
    accelBefore = accel[0:brakeTime]
    accelAfter = accel[brakeTime:len(accel)]
    if brakeTime == len(accel):
        accelAfter = 0
    meanAccel = np.mean(accelAfter)    
    
    likelihood = sum(pow(accelBefore,2))+sum(pow(accelAfter-meanAccel,2))
    likelihood += brakeLikelihood*sensorNoiseSD
    
    return [meanAccel, likelihood]
    

def movePhysics(vstate,speed,accel,predictTime):
    newstate = vstate.copy()
    dT = predictTime - vstate.time
    newstate.speed = speed + accel*dT
    if newstate.speed >= 0:
        newstate.x = vstate.x + (speed*dT + accel/2*dT*dT)*np.sin(vstate.angle)
        newstate.y = vstate.y + (speed*dT + accel/2*dT*dT)*np.cos(vstate.angle)
    else:
        newstate.speed = 0.0
        newstate.x = vstate.x + speed*speed/accel/2 * np.sin(vstate.angle)
        newstate.y = vstate.y + speed*speed/accel/2 * np.cos(vstate.angle)
    newstate.time = predictTime
    return newstate


def RearEndPredict(sensedV,trueV,trueEgo,predictTimes):
    
    filterLen = min(6, sensedV.shape[0])  
    
    # first check for braking    
    speed = np.array(sensedV.loc[:,'speed'])
    accel = np.diff(speed, 1)
    modelAccels = []
    modelHoods = []
    if filterLen <= 1:
        chosenAccel=0
        chosenTime=0
    else:
        for brakeTime in np.arange(len(accel)-filterLen,len(accel)):
            [thisAccel,thisHood] = brakeLikelihood(speed, accel, brakeTime)
            modelAccels += [thisAccel]
            modelHoods += [thisHood]
        chosenTime = np.argmin(modelHoods)
        chosenAccel = modelAccels[chosenTime]
    
    # find current speed
    filteredSpeed = speed[len(speed)-filterLen:]
    for time in range(filterLen):
        filteredSpeed[time] += chosenAccel*(filterLen-1-max(chosenTime,time))
    chosenSpeed = np.mean(filteredSpeed)
    chosenAccel = chosenAccel / .1 # TODO make this more flexible

    # now you can predict
    # return a dataframe of answers
    vcurrentState = sensedV.iloc[sensedV.shape[0]-1]
    returnedStates = sensedV[sensedV['time']<0] # empty
    for time in predictTimes:
        vNextState = movePhysics(vcurrentState,chosenSpeed,chosenAccel,time)
        returnedStates = returnedStates.append(vNextState)
    return returnedStates
