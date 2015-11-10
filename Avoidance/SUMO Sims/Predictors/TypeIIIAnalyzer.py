# -*- coding: utf-8 -*-
"""
Analyzer for Rear-End:
    adds some error after-fact to future trajectory
    predicts collision
    make table with info (speeds at end, etc)
"""
import numpy as np
import pandas as pd
import sys,os
sys.path.append(os.path.dirname(__file__)[:-len("/Analyzers")])
import collisionCheck

vehicleFolder = os.path.realpath("../Results")
sensorFolder = os.path.realpath("../Sensor Results")
outputFolder = os.path.realpath("../Analysis")

error = 2.0 # meters
timePredict = 3 # seconds


# get sensor dataset
simName = "highway"
nsims = 100
egoID = 'v1'

nTP = 0.0
nT = 0.0
nP = 0.0

for simIndex in range(nsims):
    vehicleFile = vehicleFolder + "/" + simName + np.str(simIndex+1) + ".csv"
    sensorFile = sensorFolder + "/" + simName + np.str(simIndex+1) + ".csv"
    outputFile = outputFolder + "/" + simName + np.str(simIndex+1) + ".csv"
    
    vehicleData = pd.read_table(vehicleFile, sep=";")
    vehicleData = vehicleData[["timestep_time","vehicle_id","vehicle_x","vehicle_y",
                   "vehicle_angle","vehicle_speed"]]
    vehicleData.columns = ["time","vehID","x","y","angle","speed"]  
    vehicleData['length'] = pd.Series(4.0, index=vehicleData.index)
    vehicleData['width'] = pd.Series(2.0, index=vehicleData.index)
    #sensorData = pd.read_table(vehicleFile,sep=";")
    
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
    trueCollisions = []
    lastWasCrash = False
    for ind in range(len(timeList)):
        thisWasCrash = False
        currentData = timeVehicleData[ind]
        isEgo = currentData['vehID']==egoID
        if np.any(isEgo) and not np.all(isEgo):        
            egoVehicle = currentData[isEgo].iloc[0]              
            otherVehs = currentData[isEgo==False]
            currentCollides = currentData[isEgo==False].apply(collisionCheck.check,
                            axis=1, args=(egoVehicle,))
            thisWasCrash = np.any(currentCollides)
        if thisWasCrash and not lastWasCrash: # only store first instance of collision
            trueCollisions += [timeList[ind]]
        lastWasCrash = thisWasCrash
        
    
    # easiest prediction model, true state with error :
    vehicleData['x'] = vehicleData['x'] + np.random.uniform(-error, error, vehicleData.shape[0])
    vehicleData['y'] = vehicleData['y'] + np.random.uniform(-error, error, vehicleData.shape[0])
    
    # repeat from earlier
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
    predCollisions = []
    lastWasCrash = False
    for ind in range(len(timeList)):
        thisWasCrash = False
        currentData = timeVehicleData[ind]
        isEgo = currentData['vehID']==egoID
        if np.any(isEgo) and not np.all(isEgo):        
            egoVehicle = currentData[isEgo].iloc[0]
            otherVehs = currentData[isEgo==False]            
            currentCollides = currentData[isEgo==False].apply(collisionCheck.check,
                            axis=1, args=(egoVehicle,))
            thisWasCrash = np.any(currentCollides)
        if thisWasCrash and not lastWasCrash: # only store first instance of collision
            predCollisions += [timeList[ind]]
        lastWasCrash = thisWasCrash
        
    
    tolerance = .2
    trueCollisions = np.array(trueCollisions)
    predCollisions = np.array(predCollisions)
    for predcoll in predCollisions:
        if np.any(np.abs(trueCollisions - predcoll) <= tolerance):
            nTP += 1
    nT += len(trueCollisions)
    nP += len(predCollisions)
            
if nTP > 0:
    FN = (nT-nTP)/nT
    FP = (nP-nTP)/nP
    
    
        