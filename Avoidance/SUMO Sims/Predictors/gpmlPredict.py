# -*- coding: utf-8 -*-
"""
designed to predict a rearEndBrake controlled vehicle
11/10/15
"""
# Install filterpy
# http://pythonhosted.org/filterpy/
# https://github.com/rlabbe/filterpy

import numpy as np
import pandas as pd
from sklearn.gaussian_process import GaussianProcess as GP

def initPredictor(n_feat, *args):
#    gpX =  GP(theta0=np.ones(n_feat)*args[0],
#             thetaL=np.ones(n_feat)*args[1],
#             thetaU=np.ones(n_feat)*args[2])
#    gpY =  GP(theta0=np.ones(n_feat)*args[0],
#             thetaL=np.ones(n_feat)*args[1],
#             thetaU=np.ones(n_feat)*args[2])
    gpX = GP()
    gpY = GP()
             
    return gpX, gpY

def GpmlPredict(vData,predictTimes, *args):
    n = vData.shape[0] # number of data
    filterLen = min(10, vData.shape[0])
    
    currState = vData.iloc[vData.shape[0]-1] # Current state (last data)
    currTime = currState['time'] # Current time
    # Copy temporary state for prediction
    returnedStates = vData[vData['time']<0] # empty state to return
    currXY = np.array([currState['x'], currState['y']])
    
    if filterLen < 3:
        vx_pred = np.sin(currState['speed'])
        vy_pred = -np.cos(currState['speed'])
        for time in predictTimes:
            # save predicted state
            newState = currState.copy()
            newState.x = currState.x+(time-currTime)*vx_pred
            newState.y = currState.y+(time-currTime)*vy_pred
            newState.speed = np.sqrt(vx_pred**2+vy_pred**2)
            newState.time = time
            returnedStates = returnedStates.append(newState)
            
            currState = newState[:]
            currTime = currState['time'] # Current times
    else:
        xDiff = vData.iloc[vData.shape[0]-1].x-vData.iloc[vData.shape[0]-2].x
        yDiff = vData.iloc[vData.shape[0]-1].y-vData.iloc[vData.shape[0]-2].y
        if (xDiff == 0) and (yDiff == 0): # Vehicle is not moving
            for time in predictTimes:
                # save predicted state
                newState = currState.copy()
                newState.time = time
                returnedStates = returnedStates.append(newState)
        else:
            
            xyTrain = np.array(vData[['x','y']])[n-filterLen:n,:] # position data
            vTemp = np.array(vData['speed'])[n-filterLen:n]
            vTemp2 = np.array(vData['angle'])[n-filterLen:n]
            vTrain = np.zeros((filterLen,2)) # velocity data
            vTrain[:,0] = np.multiply(vTemp,np.sin(vTemp2))
            vTrain[:,1] = -np.multiply(vTemp,np.cos(vTemp2))
            
            gpX = args[0]
            gpY = args[1]
            gpX.fit(xyTrain, vTrain[:,0])
            gpY.fit(xyTrain, vTrain[:,1])
            
#            isFirst = args[1]
#            timeNext = args[2]
            
            for time in predictTimes:
                vx_pred = gpX.predict(currXY)
                vy_pred = gpY.predict(currXY)
                
                # save predicted state
                newState = currState.copy()
                newState.x = currState.x+(time-currTime)*vx_pred
                newState.y = currState.y+(time-currTime)*vy_pred
                newState.speed = np.sqrt(vx_pred**2+vy_pred**2)
                newState.time = time
                returnedStates = returnedStates.append(newState)
                
                currState = newState[:]
                currTime = currState['time'] # Current time

    return returnedStates
