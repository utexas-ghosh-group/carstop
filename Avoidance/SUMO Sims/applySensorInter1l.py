# -*- coding: utf-8 -*-
"""
Take vehicle position/vel results and return the data from the vehicle's sensors

Only prints to csv,
   changing it to a function for the analyzer to call would not be hard
2/8/16
"""

import numpy as np
import pandas as pd
import Sensors
import os
from subprocess import call as terminalCall
from applySensor import applySensor

inputFolder = os.path.realpath("Results")
outputFolder = os.path.realpath("Sensor Results")

nsims = 40
sensorToUse = Sensors.FrontRadar
vehicleIDtoSeek = 'ego'

options = []
for letter in ['b']:
    for maxCommunicationRange in [60., 150.]:
        for noiseLevel in [0., .5, 1.]:
            for angle in [.125, .25, .5]:
                options += [[letter, maxCommunicationRange, noiseLevel, angle]]
                
for option in options:
    letter, maxCommunicationRange, noiseLevel, angle = option
    simName = "inter1l/constant_" + letter
    outputName = "inter1l/constant_" + letter
    outputName = outputName + "_" + str(int(maxCommunicationRange))
    outputName = outputName + "_" + str(noiseLevel) + "_" + str(angle)
    terminalCall(['mkdir','-p',outputFolder+"/"+outputName])

    for simIndex in range(nsims):
        
        inFile = inputFolder+"/"+simName+"/"+np.str(simIndex+1)+".csv"
        vdata = pd.read_table(inFile, sep=',')
        
        sensorTable = applySensor(vdata, vehicleIDtoSeek, sensorToUse,
                                  [noiseLevel,
                                   maxCommunicationRange, 0., angle],
                                   sensorToUse, [noiseLevel, 100., 0., 1.])
        
        outFile = outputFolder+"/"+outputName+"/"+np.str(simIndex+1)+".csv"
        sensorTable.to_csv(outFile, index = False)