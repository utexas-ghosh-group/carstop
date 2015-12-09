# -*- coding: utf-8 -*-
"""
Runs a predictor code and returns the output
11/10/15
"""
import numpy as np
import pandas as pd
import os
import collisionCheck
import Predictors

vehicleFolder = os.path.realpath("Results")
sensorFolder = os.path.realpath("Sensor Results")
outputFolder = os.path.realpath("Analysis")


# parameters to change:
simName = "rearEnd"
nsims = 25
egoID = 'ego'
minPredict = 3 # seconds
maxPredict = 5 # seconds
trajectoryPredictor = Predictors.GenericNoisePredict
VEHsize = (5.,2.)

truth = []
pred = []


for simIndex in range(nsims):
    print "- nsim: " + str(simIndex+1) +" / " +str(nsims)
    #simIndex=2
    vehicleFile = vehicleFolder + "/" + simName + np.str(simIndex+1) + ".csv"
    sensorFile = sensorFolder + "/" + simName + np.str(simIndex+1) + ".csv"
    outputFile = outputFolder + "/" + simName + np.str(simIndex+1) + ".csv"
    
    vehicleData = pd.read_table(vehicleFile,sep=',') # new data read
    # old data read:
    #vehicleData = pd.read_table(vehicleFile, sep=";")
    #vehicleData = vehicleData[["timestep_time","vehicle_id","vehicle_x","vehicle_y",
    #               "vehicle_angle","vehicle_speed"]]
    #vehicleData.columns = ["time","vehID","x","y","angle","speed"]
    vehicleData['length'] = pd.Series(VEHsize[0], index=vehicleData.index)
    vehicleData['width'] = pd.Series(VEHsize[1], index=vehicleData.index)  
    
    sensorData = pd.read_table(sensorFile,sep=",")
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
    
    # find collision times:
    trueCollision = -1
    for ind in range(len(timeList)):
        currentData = timeVehicleData[ind]
        isEgo = currentData['vehID']==egoID
        if np.any(isEgo) and not np.all(isEgo):        
            egoVehicle = currentData[isEgo].iloc[0]              
            otherVehs = currentData[isEgo==False]
            currentCollides = currentData[isEgo==False].apply(collisionCheck.check,
                            axis=1, args=(egoVehicle,))
            # only store first instance of collision
            if  np.any(currentCollides) and trueCollision < 0:
                trueCollision = timeList[ind]
        
    predictedCollision = -1
    
    for time in timeList:
        currSensorData = sensorData[sensorData['time'] <= time]
        # rearrange sensor data into dict with names
        allVehicles = {}
        allSensors = {}
        otherIDs = np.unique(currSensorData['vehID'])
        for vehID in otherIDs:
            allVehicles[vehID] = vehicleData[vehicleData['vehID']==vehID]
            allSensors[vehID] = currSensorData[currSensorData['vehID']==vehID]
        egoVehicle = vehicleData[vehicleData['vehID']==egoID]
        
        predictZone = (egoVehicle['time'] > time+minPredict)&(
                        egoVehicle['time'] < time+maxPredict)
        predictTimes = list( egoVehicle[predictZone]['time'] )      
        
        # for each other vehicle, predict path  
        for vehID in otherIDs:
            predictedPath = trajectoryPredictor(allSensors[vehID],
                                                allVehicles[vehID],
                                                egoVehicle, predictTimes)
            # check for collision
            for prediction in range(len(predictTimes)):
                thisEgo = egoVehicle[predictZone].iloc[prediction]
                thisPath = predictedPath.iloc[prediction]
                if collisionCheck.check(thisEgo, thisPath) and predictedCollision < 0:
                    predictedCollision = predictTimes[prediction]
    
    truth += [trueCollision]
    pred += [predictedCollision]


# basic scoring
nTP = 0.0
nT = 0.0
nP = 0.0
tolerance = .5
for sim in range(nsims):
    trueCollision = truth[sim]
    predictedCollision = pred[sim]
    if trueCollision > 0:
        nT += 1
    if predictedCollision > 0:
        nP += 1
    if (trueCollision > 0 and predictedCollision > 0 and
        np.abs(predictedCollision - trueCollision) <= tolerance):
            nTP += 1


# scoring            
if nTP > 0:
    FN = (nT-nTP)/nT
    FP = (nP-nTP)/nP    
    
print "Precision: " +str(nTP/float(nP))
print "Recall: " + str(nTP/float(nT))