# -*- coding: utf-8 -*-
"""
Runs a predictor code and returns the output
1/24/16
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
simName = "inter1l/q"
sensorName = "inter1l/q_200_0.0_"
nsims = 10
egoID = 'ego'
minPredict = 2.5 # seconds
maxPredict = minPredict+1 # seconds
#trajectoryPredictor = Predictors.GenericNoisePredict
#trajectoryPredictorSettings = [ 2. ]
#egoPredictor = Predictors.GenericNoisePredict
#egoPredictorSettings = [ 2. ]
#Q = np.diag([2., 2., 1., .5])
#R = np.diag([3., 3., 1., .3])
#trajectoryPredictor = Predictors.KalmanPredict_CV
#trajectoryPredictorSettings = [ .1, Q, R ]
#egoPredictor = Predictors.KalmanPredict_CV
#egoPredictorSettings = [ .1, Q, R ]
#Q = np.diag([2., 2., 1., .5, .3, .3])
#R = np.diag([3., 3., 1., .3])
#trajectoryPredictor = Predictors.KalmanPredict_CA_angle
#trajectoryPredictorSettings = [ .1, Q, R ]
#egoPredictor = Predictors.KalmanPredict_CA_angle
#egoPredictorSettings = [ .1, Q, R ]
Q = np.diag([2., 2.])
R = np.diag([1., 1.])
trajectoryPredictor = Predictors.KalmanPredict_line
trajectoryPredictorSettings = [ .1, 'N2S', Q, R ]
egoPredictor = Predictors.KalmanPredict_line
egoPredictorSettings = [ .1, 'S2W', Q, R ]
#trajectoryPredictor = Predictors.RearEndKalman
#trajectoryPredictorSettings = [ .1, 1. ]
#egoPredictor = Predictors.GenericNoisePredict
#egoPredictorSettings = [ 1. ]

VEHsize = (5.,2.)

# Input arguments for Predictor function
# egoVehicle: vehicle's data until current time
# predictTimes: time steps to predict
# *args: Can send additional arguments based on Predictor's type

paramFile = paramFolder+ "/" + simName + "_param.csv"
paramTruth = pd.read_csv(paramFile)

truth = paramTruth["Collision Time"].tolist()
truthVehID = paramTruth["Colliding Vehicle"].tolist()
pred = []
predVehID = []


for simIndex in range(nsims):
    print "- nsim: " + str(simIndex+1) +" / " +str(nsims)
    print " Loading and modifying data ..."
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
    for time in timeList[1:]:
        # load data until current time
        currSensorData = sensorData[sensorData['time'] <= time]
        egoVehicle = egoVehicleTrue[egoVehicleTrue['time'] <= time]
        # rearrange sensor data into dict with names
        allSensors = {} # All sensored data until current time
        otherIDs = np.unique(currSensorData['vehID'])
        for vehID in otherIDs:
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
print str(nT)+" collisions, "+str(nP)+" predictions"
print "correct collisions: " + str(nTP)
print "correct safe: "+ str(nTN)
print "warnings ahead of time: "+str(ahead)
print "warnings behind time: "+str(behind)