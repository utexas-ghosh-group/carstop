# -*- coding: utf-8 -*-
"""
Take vehicle position/vel results and return the data from the vehicle's sensors
Only returns data sensed by one vehicle, the designated ego vehicle
1/28/16
"""

import numpy as np
import pandas as pd
#import Sensors
#from constants import VState

def applySensor(vdata, egoVehicleID, sensorToUse, sensorSettings,
                egoSensorToUse = None, egoSensorSettings=None):
    
    # read a result file - old
    #vdata = pd.read_table(inFile, sep=";")
    #vdata = vdata[["timestep_time","vehicle_id","vehicle_x","vehicle_y",
    #                   "vehicle_angle","vehicle_speed"]]
    #vdata.columns = ["time","vehID","x","y","angle","speed"]            
    
    # rearrange data into panel of dataframes (by time)
    timeSortedData = {}
    currentTime = vdata.iloc[0]["time"]
    timeList = [currentTime] # keeping ordered time seperately
    lastIndex = 0
    for ind in range(vdata.shape[0]):
        if vdata.iloc[ind]["time"] > currentTime:
            timeSortedData[currentTime] = vdata.iloc[lastIndex:ind]
            lastIndex = ind
            currentTime = vdata.iloc[ind]["time"]
            timeList = timeList + [currentTime]
        if ind == vdata.shape[0] - 1:
            timeSortedData[currentTime] = vdata.iloc[lastIndex:ind+1]
    
    # for each time, check for egoVehicleID if specified
    sensorData = []
    for time in timeList:
        currentData = timeSortedData[time]
        egoLoc = currentData['vehID'] == egoVehicleID
        if np.any(egoLoc):
            # run sensor on each vehicle
            egoVehicle = currentData[egoLoc].iloc[0]
            sensor = sensorToUse(egoVehicle, False, *sensorSettings)
            currentData[egoLoc == False].apply(sensor.addObstacle, axis = 1)
            if egoSensorToUse is None:
                sensorData += [pd.DataFrame(sensor.getObstacles())]
            else:
                egoSensor = egoSensorToUse(egoVehicle, False,
                                           *egoSensorSettings)
                egoSensor.addObstacle(egoVehicle)
                allObstacles = sensor.getObstacles() + egoSensor.getObstacles()
                sensorData += [pd.DataFrame(allObstacles)]
    
    # print all sensing instances into a csv
    sensorTable = pd.DataFrame([])
    for sensingInstance in sensorData:
        sensorTable = sensorTable.append(sensingInstance)
    return sensorTable