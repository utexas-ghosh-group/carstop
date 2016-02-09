# -*- coding: utf-8 -*-
"""
rather simple caller for rear end simulations
2/8/16
"""

import numpy as np
import pandas as pd
import Sensors
import os
from subprocess import call as terminalCall
from applySensor import applySensor
#from constants import VState

inputFolder = os.path.realpath("Results")
outputFolder = os.path.realpath("Sensor Results")

nsims = 30
sensorToUse = Sensors.DSRC
#sensorToUse = Sensors.FrontRadar
vehicleIDtoSeek = 'ego'

for moveType in ['rtest']:
    for noiseLevel in [0., 1.]:
        simName = "rearEnd/" + moveType
        outputName = "rearEnd/" + moveType
        outputName = outputName + "_" + str(noiseLevel)
        terminalCall(['mkdir','-p',outputFolder+"/"+outputName])
    
        for simIndex in range(nsims):
            
            inFile = inputFolder+"/"+simName+"/"+np.str(simIndex+1)+".csv"
            vdata = pd.read_table(inFile, sep=',')
            
            sensorTable = applySensor(vdata, vehicleIDtoSeek, sensorToUse,
                                      [noiseLevel,
                                       200., 0.],
                                       sensorToUse, [noiseLevel, 200., 0.])
            
            outFile = outputFolder+"/"+outputName+"/"+np.str(simIndex+1)+".csv"
            sensorTable.to_csv(outFile, index = False)