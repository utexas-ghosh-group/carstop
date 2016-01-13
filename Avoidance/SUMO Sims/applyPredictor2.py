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
paramFolder = os.path.realpath("Parameters")
#simName = "rearEnd"


# parameters to change:
simName = "inter1l/a"
sensorName = "inter1l/a_100_1.0_"
nsims = 40
egoID = 'ego'
minPredict = 2.5 # seconds
maxPredict = minPredict+1 # seconds
trajectoryPredictor = Predictors.GenericNoisePredict # Trajectory Predictor for other vehicle
trajectoryPredictorSettings = [ 2. ]
egoPredictor = Predictors.GenericNoisePredict # Trajectory Predictor for ego vehicle
egoPredictorSettings = [ 2. ]
VEHsize = (5.,2.)

# Input arguments for Predictor function
# egoVehicle: vehicle's data until current time
# predictTimes: time steps to predict
# *args: Can send additional arguments based on Predictor's type

paramFile = paramFolder+ "/" + simName + "_param.csv"
paramTruth = pd.read_csv(paramFile)

truth = paramTruth["Collision Time"].tolist()
truthVehID = ['alter']*len(truth) # Crashed vehicle's ID for corresponding time (temporary setting)
pred = []
predVehID = []


for simIndex in range(nsims):
    print "- nsim: " + str(simIndex+1) +" / " +str(nsims)
    print " Loading and modifying data ..."
    #simIndex=2
    vehicleFile = vehicleFolder + "/" + simName + np.str(simIndex+1) + ".csv"
    sensorFile = sensorFolder + "/" + sensorName + np.str(simIndex+1) + ".csv"
    
    vehicleData = pd.read_table(vehicleFile,sep=',') # new data read
    # old data read:
    #vehicleData = pd.read_table(vehicleFile, sep=";")
    #vehicleData = vehicleData[["timestep_time","vehicle_id","vehicle_x","vehicle_y",
    #               "vehicle_angle","vehicle_speed"]]
    #vehicleData.columns = ["time","vehID","x","y","angle","speed"]
    vehicleData['length'] = pd.Series(VEHsize[0], index=vehicleData.index)
    vehicleData['width'] = pd.Series(VEHsize[1], index=vehicleData.index)  
    
    try:
        sensorData = pd.read_table(sensorFile,sep=",")
    except ValueError: # empty file
        pred += [-1]
        predVehID += ['']
        continue
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
#        isEgo = currentData['vehID']==egoID
#        if np.any(isEgo) and not np.all(isEgo):        
#            egoVehicle = currentData[isEgo].iloc[0]              
#            otherVehs = currentData[isEgo==False]
#            currentCollides = currentData[isEgo==False].apply(collisionCheck.check,
#                            axis=1, args=(egoVehicle,))
#            # only store first instance of collision
#            if  np.any(currentCollides) and trueCollision < 0:
#                trueCollision = timeList[ind]
    
    print " Predicting collision ..."    
    predictedCollision = -1
    predictedCollisionVehID = ''
    
    # set up predictors, with true data for all time steps
    egoVehicleTrue = vehicleData[vehicleData['vehID']==egoID]
    egoPredictors = egoPredictor(egoVehicleTrue, *egoPredictorSettings)
    predictors = {}    
    for vehID in np.unique(vehicleData['vehID']):
        predictors[vehID] = trajectoryPredictor(
                                vehicleData[vehicleData['vehID']==vehID],
                                *trajectoryPredictorSettings)
    for time in timeList:
        # load data until current time
        currSensorData = sensorData[sensorData['time'] <= time]
        egoVehicle = egoVehicleTrue[egoVehicleTrue['time'] <= time]
        # rearrange sensor data into dict with names
        allVehicles = {} # All vehicle data for sensored vehicle
        allSensors = {} # All sensored data until current time
        otherIDs = np.unique(currSensorData['vehID'])
        for vehID in otherIDs:
            allVehicles[vehID] = vehicleData[vehicleData['vehID']==vehID]
            allSensors[vehID] = currSensorData[currSensorData['vehID']==vehID]
        
        # Time to predict: current Time+min ~ current Time+max (s)
        # time steps are extracted from ego Vehicle
        predictZone = (egoVehicleTrue['time'] >= time+minPredict)&(
                        egoVehicleTrue['time'] <= time+maxPredict)
        predictTimes = list( egoVehicleTrue[predictZone]['time'] )      
        
        # for ego vehicle, predict path
        egoPredicted = egoPredictors.predict(egoVehicle, predictTimes)
                
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
                if collisionCheck.check(thisEgo, thisPath) and predictedCollision < 0:
                    predictedCollision = predictTimes[prediction]
                    predictedCollisionVehID = vehID
                    # !!!!!!!! Think about how to deal with multiple collision
                    
#    truth += [trueCollision]r
    pred += [predictedCollision]
    predVehID += [predictedCollisionVehID]


# basic scoring
nTP = 0.0
nT = 0.0
nP = 0.0
tolerance = 1.
for sim in range(nsims):
    trueCollision = truth[sim]
    trueVehID = truthVehID[sim]
    predictedCollision = pred[sim]
    predictedVehID = predVehID[sim]
    if trueCollision > 0:
        nT += 1
    if predictedCollision > 0:
        nP += 1
    if (trueCollision > 0 and predictedCollision > 0 and trueVehID == predictedVehID and
        np.abs(predictedCollision - trueCollision) <= tolerance):
            nTP += 1


# scoring            
if nTP > 0:
    FN = (nT-nTP)/nT
    FP = (nP-nTP)/nP    
    print "Precision: " +str(nTP/float(nP))
    print "Recall: " + str(nTP/float(nT))