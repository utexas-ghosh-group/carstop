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


# parameters to change:
simName = "rearEnd"
nsims = 30
egoID = 'ego'
minPredict = 3 # seconds
maxPredict = 5 # seconds

# Choose predictor here !!!!!!
predictorChoose = 1 # 1: UKF, 2: GP

if predictorChoose == 1:
    trajectoryPredictor = Predictors.KalmanPredict # Trajectory Predictor
    initPred = Predictors.kalmanPredict.initPredictor
elif predictorChoose == 2:
    trajectoryPredictor = Predictors.GpmlPredict # Trajectory Predictor
    initPred = Predictors.gpmlPredict.initPredictor
else:
    trajectoryPredictor = Predictors.GenericNoisePredict # Trajectory Predictor=

#trajectoryPredictor = Predictors.RearEndPredict # Trajectory Predictor for other vehicle
#egoPredictor = Predictors.RearEndPredict # Trajectory Predictor for ego vehicle    

VEHsize = (5.,2.)

# Input arguments for Predictor function
# egoVehicle: vehicle's data until current time
# predictTimes: time steps to predict
# *args: Can send additional arguments based on Predictor's type

paramFile = paramFolder+ "/" + simName + "_param.csv"
paramTruth = pd.read_csv(paramFile)

truth = paramTruth["Collision Time"].tolist()
truthVehID = ['lead']*len(truth) # Crashed vehicle's ID for corresponding time (temporary setting)
pred = [] # predicted collision time
predVehID = [] # predicted collision vehicle's ID

n_state = 4 # Dimension of states
n_meas = 4 # Dimension of observations


for simIndex in range(nsims):
    print "- nsim: " + str(simIndex+1) +" / " +str(nsims)
    print " Loading and modifying data ..."
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
    predictedCollisionVehID = egoID
    
    otherIDs_all = np.unique(sensorData['vehID']).tolist()
    tStart_others = np.zeros(len(otherIDs_all)) # first time vehicle recoreded

    # Initialize Predictors (Kalman, GP)
    if predictorChoose == 1:
        pred_ego = Predictors.kalmanPredict.initPredictor(n_state, n_meas,0.1)
        pred_others = [None]*len(otherIDs_all)
    elif predictorChoose == 2:
        pred_egoX, pred_egoY = Predictors.gpmlPredict.initPredictor(2,0.1,0.001,1.0)
        pred_othersX = [None]*len(otherIDs_all)
        pred_othersY = [None]*len(otherIDs_all)
    tStart_ego = vehicleData[vehicleData['vehID']==egoID]['time'].iloc[0]
    for iv in range(len(otherIDs_all)):
        if predictorChoose == 1:
            pred_others[iv] = Predictors.kalmanPredict.initPredictor(n_state, n_meas,0.1)
        elif predictorChoose == 2:
            pred_othersX[iv], pred_othersY[iv] = Predictors.gpmlPredict.initPredictor(2,0.1,0.001,1.0)
        tStart_others[iv] = vehicleData[vehicleData['vehID']==otherIDs_all[iv]]['time'].iloc[0]
    
    # Save ego vehicle's true data for all time steps
    egoVehicleTrue = vehicleData[vehicleData['vehID']==egoID]        
    for time in timeList:
#        if time == timeList[0]: # First time step (No previous info)
            
        if predictedCollision >= 0:
            break # Save only first predicted collision
        # load data until current time
        currSensorData = sensorData[sensorData['time'] <= time]
        egoVehicle = egoVehicleTrue[egoVehicleTrue['time'] <= time]
        # Load current time data
#        currSensorData = sensorData[sensorData['time'] == time]
#        egoVehicle = egoVehicleTrue[egoVehicleTrue['time'] == time]
        # rearrange sensor data into dict with names
        allVehicles = {} # All vehicle data for sensored vehicle
        allSensors = {} # All sensored data until current time
        otherIDs = np.unique(currSensorData['vehID']).tolist()
        for vehID in otherIDs:
            allVehicles[vehID] = vehicleData[vehicleData['vehID']==vehID]
            allSensors[vehID] = currSensorData[currSensorData['vehID']==vehID]
        
        # Time to predict: current Time+min ~ current Time+max (s)
        # time steps are extracted from ego Vehicle
        predictZone = (egoVehicleTrue['time'] >= time+minPredict)&(
                        egoVehicleTrue['time'] <= time+maxPredict)
        predictTimes = list( egoVehicleTrue[predictZone]['time'] )      
        
        # for ego vehicle, predict path
#        egoPredicted = egoPredictor(egoVehicle, predictTimes,
#                                    egoVehicleTrue) # genericNoisePredict
        if time < timeList[len(timeList)-1]:
            timeNext = timeList[timeList.index(time)+1] # Next time step
            
        if time == tStart_ego: # first time step (no previoius info)
            if predictorChoose == 1:
                pred_ego.x[0] = egoVehicle.iloc[np.shape(egoVehicle)[0]-1].x
                pred_ego.x[1] = egoVehicle.iloc[np.shape(egoVehicle)[0]-1].y
                pred_ego.x[2] = egoVehicle.iloc[np.shape(egoVehicle)[0]-1].speed
                pred_ego.x[3] = egoVehicle.iloc[np.shape(egoVehicle)[0]-1].angle
                egoPredicted = trajectoryPredictor(egoVehicle, predictTimes,
                                                   pred_ego, True, timeNext) # kalmanPredict
            elif predictorChoose == 2:
                egoPredicted = trajectoryPredictor(egoVehicle, predictTimes,
                                                   pred_egoX, pred_egoY, True, timeNext) # gpmlPredict
            else:
                egoPredicted = egoPredicted = trajectoryPredictor(egoVehicle, predictTimes,
                                                                  egoVehicleTrue) # genericNoisePredict
        else:
            if predictorChoose == 1:
                egoPredicted = trajectoryPredictor(egoVehicle, predictTimes,
                                                   pred_ego, False, timeNext) # kalmanPredict
            elif predictorChoose == 2:
                egoPredicted = trajectoryPredictor(egoVehicle, predictTimes,
                                                   pred_egoX, pred_egoY, False, timeNext) # gpmlPredict
            else:
                egoPredicted = egoPredicted = trajectoryPredictor(egoVehicle, predictTimes,
                                                                  egoVehicleTrue) # genericNoisePredict
                
        # for each other vehicle, predict path  
        for vehID in otherIDs:
            iv = otherIDs_all.index(vehID)
#            predictedPath = trajectoryPredictor(allSensors[vehID], predictTimes, 
#                                                allVehicles[vehID]) # genericNoisePredict
            if time == tStart_others[iv]: # first time step (no previoius info)
                if predictorChoose == 1:
                    pred_others[iv].x[0] = allSensors[vehID].iloc[np.shape(allSensors[vehID])[0]-1].x
                    pred_others[iv].x[1] = allSensors[vehID].iloc[np.shape(allSensors[vehID])[0]-1].y
                    pred_others[iv].x[2] = allSensors[vehID].iloc[np.shape(allSensors[vehID])[0]-1].speed
                    pred_others[iv].x[3] = allSensors[vehID].iloc[np.shape(allSensors[vehID])[0]-1].angle
                    predictedPath = trajectoryPredictor(allSensors[vehID], predictTimes,
                                                        pred_others[iv], True, timeNext) # kalmanPredict
                elif predictorChoose == 2:
                    predictedPath = trajectoryPredictor(allSensors[vehID], predictTimes,
                                                       pred_othersX[iv], pred_othersY[iv], True, timeNext) # gpmlPredict
                else:
                    predictedPath = trajectoryPredictor(allSensors[vehID], predictTimes, 
                                                        allVehicles[vehID]) # genericNoisePredict
            else:
                if predictorChoose == 1:
                    predictedPath = trajectoryPredictor(allSensors[vehID], predictTimes,
                                                        pred_others[iv], False, timeNext) # kalmanPredict
                elif predictorChoose == 2:
                    predictedPath = trajectoryPredictor(allSensors[vehID], predictTimes,
                                                       pred_othersX[iv], pred_othersY[iv], False, timeNext) # gpmlPredict
                else:
                    predictedPath = trajectoryPredictor(allSensors[vehID], predictTimes, 
                                                        allVehicles[vehID]) # genericNoisePredict
            # check for collision
            for prediction in range(len(predictTimes)):
                thisEgo = egoPredicted.iloc[prediction]
                thisPath = predictedPath.iloc[prediction]
                if collisionCheck.check(thisEgo, thisPath) and predictedCollision < 0:
                    predictedCollision = predictTimes[prediction]
                    predictedCollisionVehID = vehID
                    # !!!!!!!! Think about how to deal with multiple collision??
                    # !!!!!!!! Save not only collision time but names ??
    
#    truth += [trueCollision]r
    pred += [predictedCollision]
    predVehID += [predictedCollisionVehID]


# basic scoring
nTP = 0.0
nT = 0.0
nP = 0.0
tolerance = .5
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
if nP > 0:    
    print "Precision: " +str(nTP/float(nP))
else:
    print "Precison: 0 (" + str(nP) + " collision predicted)"
if nT > 0:
    print "Recall: " + str(nTP/float(nT))
else:
    print "Recall: 0 (" + str(nT) + " collision occurred)"