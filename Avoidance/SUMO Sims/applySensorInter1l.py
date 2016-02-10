# -*- coding: utf-8 -*-
"""
Take vehicle position/vel results and return the data from the vehicle's sensors

Only prints to csv,
   changing it to a function for the analyzer to call would not be hard
2/9/16
"""

import numpy as np
import pandas as pd
import Sensors
import os
from subprocess import call as terminalCall
from applySensor import applySensor

inputFolder = os.path.realpath("Results")
outputFolder = os.path.realpath("Sensor Results")

nsims = 500
sensorToUse = Sensors.FrontRadar
vehicleIDtoSeek = 'ego'

l_letter = ['random_f']
l_maxCommunicationRange = [150., 100., 50., 25.]
l_noiseLevel = [0., .33, .66, 1.]
l_angle = [1., .5, .25, .125]

options = []
for letter in l_letter:
# # try every combination of options
#    for maxCommunicationRange in l_maxCommunicationRange:
#        for noiseLevel in l_noiseLevel:
#            for angle in l_angle:
#                options += [[letter, maxCommunicationRange, noiseLevel, angle]]
# # isolate each parameter, then try combination of same levels
# # each parameter must have same number of options
    for maxCommunicationRange in l_maxCommunicationRange[1:]:
        options += [[letter,maxCommunicationRange,l_noiseLevel[0],l_angle[0]]]
    for noiseLevel in l_noiseLevel[1:]:
        options += [[letter,l_maxCommunicationRange[0],noiseLevel,l_angle[0]]]
    for angle in l_angle[1:]:
        options += [[letter,l_maxCommunicationRange[0],l_noiseLevel[0],angle]]
    for k in range(len(l_angle)):
        options += [[letter,l_maxCommunicationRange[k],l_noiseLevel[k],l_angle[k]]]

for option in options:
    letter, maxCommunicationRange, noiseLevel, angle = option
    simName = "inter1l/" + letter
    outputName = "inter1l/" + letter
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
