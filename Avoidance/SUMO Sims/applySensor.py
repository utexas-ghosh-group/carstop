# -*- coding: utf-8 -*-
"""
Take vehicle position/vel results and return the data from the vehicle's sensors
Designed for simple highway example, right now only return data for one vehicle

Only prints to csv,
    changing it to a function for the analyzer to call would not be hard
"""

import numpy as np
import pandas as pd
import Sensors
import os
#from constants import VState

inputFolder = os.path.realpath("Results")
outputFolder = os.path.realpath("Sensor Results")


simName = "highway"
nsims = 30
sensorToUse = Sensors.DSRC
vehicleIDtoSeek = 'ego'


for simIndex in range(nsims):
    #simIndex = 0
    inFile = inputFolder + "/" + simName + np.str(simIndex+1) + ".csv"
    outFile = outputFolder + "/" + simName + np.str(simIndex+1) + ".csv"
    
    # read a result file
    vdata = pd.read_table(inFile, sep=";")
    vdata = vdata[["timestep_time","vehicle_id","vehicle_x","vehicle_y",
                       "vehicle_angle","vehicle_speed"]]
    # change names to conform to simulator code
    vdata.columns = ["time","vehID","x","y","angle","speed"]                  
    
    # rearrange data into panel of dataframes (by time)
    timeSortedData = {}
    currentTime = vdata.loc[0,"time"]
    timeList = [currentTime] # keeping ordered time seperately
    lastIndex = 0
    for ind in range(vdata.shape[0]):
        if vdata.loc[ind,"time"] > currentTime:
            timeSortedData[currentTime] = vdata.iloc[lastIndex:ind]
            lastIndex = ind
            currentTime = vdata.loc[ind,"time"]
            timeList = timeList + [currentTime]
        if ind == vdata.shape[0] - 1:
            timeSortedData[currentTime] = vdata.iloc[lastIndex:ind+1]
    
    # for each time, check for vehicleIDtoSeek if specified
    # run sensor on each other vehicle
    sensorData = {}
    senseTimeList = []
    for time in timeList:
        currentData = timeSortedData[time]
        egoLoc = currentData['vehID'] == vehicleIDtoSeek
        if np.any(egoLoc):
            egoVehicle = currentData[egoLoc].iloc[0]
            sensor = sensorToUse(egoVehicle, realign=False)
            currentData[egoLoc == False].apply(sensor.addObstacle, axis = 1)
            sensorData[time] = pd.DataFrame(sensor.getObstacles())
            senseTimeList = senseTimeList + [time]
    
    # print all sensing instances into a csv
    sensorTable = pd.DataFrame([])
    for time in senseTimeList:
        sensorTable = sensorTable.append(sensorData[time])
    sensorTable.to_csv(outFile, index = False)