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
from applySensor import applySensor
#from constants import VState
from optparse import OptionParser

inputFolder = os.path.realpath("Results")
outputFolder = os.path.realpath("Sensor Results")

#optParser = OptionParser()
#optParser.add_option("-a", "--range", type="float", dest="maxCommunicationRange",
#                     default = 200.)
#optParser.add_option("-b", "--noise", type="float", dest="noiseLevel",
#                     default = 0.)
#optParser.add_option("-c", "--letter", type="string", dest="letter",
#                     default = "q")
#optParser.add_option("-n","--nsims", type="int", dest="nsims", default= 10)
#(options, args) = optParser.parse_args()
#if options.noiseLevel > 1:
#    options.noiseLevel=options.noiseLevel*.01
nsims = 10
sensorToUse = Sensors.DSRC
#sensorToUse = Sensors.FrontRadar
vehicleIDtoSeek = 'ego'

for letter in ['a']:
    for maxCommunicationRange in [200.]:
        for noiseLevel in [0., 1.]:
            simName = "inter1l/" + letter
            outputName = "inter1l/" + letter
            outputName = outputName + "_" + str(int(maxCommunicationRange))
            outputName = outputName + "_" + str(noiseLevel) + "_"

            for simIndex in range(nsims):
                
                inFile = inputFolder+"/"+simName+np.str(simIndex+1)+".csv"
                vdata = pd.read_table(inFile, sep=',')
                
                sensorTable = applySensor(vdata, vehicleIDtoSeek, sensorToUse,
                                          [noiseLevel,
                                           maxCommunicationRange, 0.],
                                           sensorToUse, [noiseLevel, 100., 0.])
                
                outFile = outputFolder+"/"+outputName+np.str(simIndex+1)+".csv"
                sensorTable.to_csv(outFile, index = False)