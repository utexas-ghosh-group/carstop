# -*- coding: utf-8 -*-
"""
Runs a predictor code on sensor data and returns the output
1/27/16
"""
import numpy as np
import pandas as pd
import collisionCheck
import Predictors

VEHsize = (5.,2.)


def applyPredictor(vehicleData, sensorData, egoVehicleID, minPredict, maxPredict,
                   trajectoryPredictor, trajectoryPredictorSettings,
                   egoPredictor = Predictors.GenericNoisePredict,
                   egoPredictorSettings = [0.]):
#    print " Loading and modifying data ..."
    vehicleData['length'] = pd.Series(VEHsize[0], index=vehicleData.index)
    vehicleData['width'] = pd.Series(VEHsize[1], index=vehicleData.index)  
    
    sensorData['length'] = pd.Series(VEHsize[0], index=sensorData.index)
    sensorData['width'] = pd.Series(VEHsize[1], index=sensorData.index)
    
    # rearrange vehicle data into list of dataframes (by time)
    timeVehicleData = []
    currentTime = vehicleData.loc[0,"time"]
    timeList = [currentTime] # keeping ordered time seperately
    lastIndex = 0
    for ind in range(vehicleData.shape[0]):
        if vehicleData.loc[ind,"time"] > currentTime:
            timeVehicleData += [ vehicleData.iloc[lastIndex:ind] ]
            lastIndex = ind
            currentTime = vehicleData.loc[ind,"time"]
            timeList = timeList + [currentTime]
        if ind == vehicleData.shape[0] - 1:
            timeVehicleData += [ vehicleData.iloc[lastIndex:ind+1] ]
    
#    print " Finding collision times ..."
#    # find collision times:
#    trueCollision = -1
#    for ind in range(len(timeList)):
#        currentData = timeVehicleData[ind]
#        isEgo = currentData['vehID']==egoVehicleID
#        if np.any(isEgo) and not np.all(isEgo):        
#            egoVehicle = currentData[isEgo].iloc[0]              
#            otherVehs = currentData[isEgo==False]
#            currentCollides = currentData[isEgo==False].apply(collisionCheck.check,
#                            axis=1, args=(egoVehicle,))
#            # only store first instance of collision
#            if  np.any(currentCollides) and trueCollision < 0:
#                trueCollision = timeList[ind]F
    
    #print " Predicting collision ..."
    
    # set up predictors, with true data for all time steps
    egoVehicleTrue = vehicleData[vehicleData['vehID']==egoVehicleID]
    egoPredictors = egoPredictor(egoVehicleTrue, *egoPredictorSettings)
    predictors = {}    
    for vehID in np.unique(vehicleData['vehID']):
        predictors[vehID] = trajectoryPredictor(
                                vehicleData[vehicleData['vehID']==vehID],
                                *trajectoryPredictorSettings)
    for time in timeList[1:]:
        # load data until current time
        currSensorData = sensorData[sensorData['time'] <= time]
        egoSensor = currSensorData[currSensorData['vehID']==egoVehicleID]
        if egoSensor.shape[0]==0: # if no self-sensor, use true data
            egoSensor = egoVehicleTrue[egoVehicleTrue['time'] <= time]
        # rearrange sensor data into dict with names
        allSensors = {} # All sensored data until current time
        otherIDs = np.unique(currSensorData['vehID'])
        otherIDs = list(vid for vid in otherIDs if not vid == egoVehicleID)
        for vehID in otherIDs:
            allSensors[vehID] = currSensorData[currSensorData['vehID']==vehID]
        
        # Time to predict: current Time+min ~ current Time+max (s)
        # time steps are extracted from ego Vehicle
        predictZone = (egoVehicleTrue['time'] >= time+minPredict)&(
                        egoVehicleTrue['time'] <= time+maxPredict)
        predictTimes = list( egoVehicleTrue[predictZone]['time'] )      
        
        # for ego vehicle, predict path
        egoPredicted = egoPredictors.predict(egoSensor, predictTimes)
                
        # for each other vehicle, predict path
        for vehID in otherIDs:
            predictedPath = predictors[vehID].predict(allSensors[vehID],
                                                        predictTimes)
            if not np.any(allSensors[vehID]['time'] <= time): # break if vehicle hasn't been sensed
                break
            # check for collision
            for prediction in range(len(predictTimes)):
                thisEgo = egoPredicted.iloc[prediction]
                thisPath = predictedPath.iloc[prediction]
                if collisionCheck.check(thisEgo, thisPath):
                    return [predictTimes[prediction], vehID]
                    # !!!!!!!! Think about how to deal with multiple collision
                    
    return [-1, '']


# basic scoring
# takes lists of collision times and vehicle ID's
# -1 = no collision in this simulation
def scorePredictions(truth, truthVehID, pred, predVehID, display=True):
    nsims = len(pred)
    nTP = 0
    nTN = 0
    nT = 0
    nP = 0
    ahead = 0
    behind = 0
    tolerance = 1.
    for sim in range(nsims):
        trueCollision = truth[sim]
        trueVehID = truthVehID[sim]
        predictedCollision = pred[sim]
        predictedVehID = predVehID[sim]
        if (trueCollision > 0 and predictedCollision > 0):
            nT += 1
            nP += 1
            if np.abs(predictedCollision - trueCollision) <= tolerance:
                nTP += 1
            elif predictedCollision - trueCollision < -tolerance:
                ahead += 1
            else:
                behind += 1
        elif trueCollision > 0:
            nT += 1
        elif predictedCollision > 0:
            nP += 1
        else:
            nTN += 1
    
    # scoring
    if display:          
        print str(nT)+" collisions, "+str(nP)+" predictions"
        print "correct collisions: " + str(nTP)
        print "correct safe: "+ str(nTN)
        print "warnings ahead of time: "+str(ahead)
        print "warnings behind time: "+str(behind)
    return [nT, nP, nTP, nTN, ahead, behind]