# -*- coding: utf-8 -*-
"""
Take vehicle position/vel results and return the data from the vehicle's sensors

Only prints to csv,
   changing it to a function for the analyzer to call would not be hard
3/28/16
"""

import numpy as np
import pandas as pd
from subprocess import call as terminalCall
import os, sys
sys.path.append(os.path.realpath('..'))

import Sensors
from applySensor import applySensor

inputFolder = os.path.realpath("../Results/RR")
outputFolder = os.path.realpath("../Sensor Results/RR")

nsims = 5000
trackLength = 1300.
sensorToUse = Sensors.RSUarray

l_RSUspacing = [200.]
l_commdist = [1100.]

options = []
 # try every combination of options
for RSUspacing in l_RSUspacing:
    for commdist in l_commdist:
        options += [[RSUspacing, commdist]]

for option in options:
    RSUspacing, commdist = option
    RSUx = np.arange(np.random.uniform(-RSUspacing,0), trackLength+RSUspacing,
                     RSUspacing)
    #RSUarray = list(([RSUx[i],-3.2,np.pi/2] for i in range(len(RSUx))))
    #RSUarray += list(([RSUx[i],10.2,-np.pi/2] for i in range(len(RSUx))))
    RSUarray = list(([unitx, 2., np.pi] for unitx in RSUx))
    RSUarray += list(([unitx, 5.2, 0] for unitx in RSUx))
    
    # create folder for results
    outputName = "V2Ilong"
    outputName = outputName + "_" + str(int(RSUspacing))
    outputName = outputName + "_" + str(int(commdist))
    if os.name == 'posix': # unix
        terminalCall(['mkdir','-p',outputFolder+"/"+outputName])
    else:
        terminalCall(['mkdir',os.path.realpath(outputFolder+"/"+outputName)], shell=True)

    for simIndex in range(nsims):
        
        inFile = inputFolder+"/"+np.str(simIndex+1)+".csv"
        vdata = pd.read_table(inFile, sep=',')
        
        sensorTable = applySensor(vdata, 'Passing', Sensors.RSUarray,
                                  [RSUarray, 0., 0., commdist],
                                  Sensors.RSUarray, [RSUarray, 0., 0., commdist])
#        sensorTable = applySensor(vdata, 'Passing', Sensors.DSRC,
#                                  [0., commdist, 0.], Sensors.IdealSensor, [])
#        startTable = applySensor(vdata.iloc[:3],'Passing', Sensors.DSRC,
#                                 [0.,590.,0.],
#                                 Sensors.IdealSensor, [])
#        sensorTable = pd.concat([startTable, sensorTable])
        
        sensorTable.drop('y', axis=1, inplace=True)
        sensorTable.drop('angle', axis=1, inplace=True)
        sensorTable.columns = ['Time','Sender.ID','Position','Speed','Acceleration']
        sensorTable['Time.Sent'] = sensorTable['Time']
        outFile = outputFolder+"/"+outputName+"/"+np.str(simIndex+1)+".csv"
        sensorTable.to_csv(outFile, index = False)
    print "option complete"