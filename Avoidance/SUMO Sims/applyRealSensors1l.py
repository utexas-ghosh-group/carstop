# -*- coding: utf-8 -*-
"""
using the Delphi ERS and common DSRC values, plus a simple combination
2/9/16
"""

import numpy as np
import pandas as pd
import Sensors
import Predictors
import os
from applySensor import applySensor
from applyPredictor import applyPredictor, scorePredictions

vehicleFolder = os.path.realpath("Results")
outputFolder = os.path.realpath("Analysis")
paramFolder = os.path.realpath("Parameters")

simName = "inter1l/random_b"
nsims = 500

sensor1 = Sensors.DSRC
sensor1Settings = [1., 200., 0.]
egoSensor1 = Sensors.IdealSensor
egoSensor1Settings = []
sensor2 = Sensors.DelphiESR
sensor2Settings = [False]
egoSensor2 = Sensors.IdealSensor
egoSensor2Settings = []
sensor3 = Sensors.DelphiESR
sensor3Settings = [True]
egoSensor3 = Sensors.IdealSensor
egoSensor3Settings = []

Q = np.diag([2., 2.])
R1 = np.diag([2., 1.2])
R2 = np.diag([2., 1.2])*.25
trajectoryNames = ['kline','caline']
trajectory1Predictors = [Predictors.KalmanPredict_line, Predictors.CV_line]
trajectory1Settings = [[ .1, 'N2S', Q, R1 ], ['N2S', 3.5]]
ego1Predictors = [Predictors.KalmanPredict_line, Predictors.CV_line]
ego1Settings = [[ .1, 'S2W', Q, R1 ], ['S2W', 3.5]]
trajectory2Predictors = [Predictors.KalmanPredict_line, Predictors.CV_line]
trajectory2Settings = [[ .1, 'N2S', Q, R2 ], ['N2S', 3.5]]
ego2Predictors = [Predictors.KalmanPredict_line, Predictors.CV_line]
ego2Settings = [[ .1, 'S2W', Q, np.diag([.01,.01])], ['S2W', 3.5]]
trajectory3Predictors = [Predictors.KalmanPredict_line, Predictors.CV_line]
trajectory3Settings = [[ .1, 'N2S', Q, R2 ], ['N2S', 3.5]]
ego3Predictors = [Predictors.KalmanPredict_line, Predictors.CV_line]
ego3Settings = [[ .1, 'S2W', Q, np.diag([.01,.01]) ], ['S2W', 3.5]]

vehicleIDtoSeek = 'ego'

class WriteFrame:
    def __init__(self, colnames):
        self.df = None
        self.colnames = colnames
    def add(self, newrow):
        if self.df is None:
            self.df = pd.DataFrame([newrow])
        else:
            self.df = self.df.append([newrow])
    def write(self, fileName, restart=False):
        self.df.columns =  self.colnames
        self.df.to_csv(fileName, sep=',', header=True, index=False)
        if restart:
            self.df = None
results = WriteFrame(['sensor',
                      'prediction algorithm', 'number of collisions',
                      'number of warnings', 'number of correct warnings',
                      'number of correct safe','early warnings','late warnings'])
                      
paramFile = paramFolder+ "/" + simName + "_param.csv"
paramTruth = pd.read_csv(paramFile)

truth = paramTruth["Collision Time"].tolist()
truthVehID = paramTruth["Colliding Vehicle"].tolist()
                               
for predind in range(len(trajectory1Predictors)):
    trajectoryPredictor = trajectory1Predictors[predind]
    trajectorySettings = trajectory1Settings[predind]
    egoPredictor = ego1Predictors[predind]
    egoSettings = ego1Settings[predind]
    
    pred = []
    predVehID = []
    for simIndex in range(nsims):
        inFile = vehicleFolder+"/"+simName+"/"+np.str(simIndex+1)+".csv"
        vData = pd.read_table(inFile, sep=',')
        sensor1Data = applySensor(vData, vehicleIDtoSeek, sensor1,
                            sensor1Settings, egoSensor1, egoSensor1Settings)
                                  
        predCollision, predictVehID = applyPredictor(vData, sensor1Data,
                                                     'ego', 2.5, 3.5,
                                                     trajectoryPredictor,
                                                     trajectorySettings,
                                                     egoPredictor,
                                                     egoSettings)
        pred += [predCollision]
        predVehID += [predictVehID]
        
    result = scorePredictions(truth, truthVehID, pred, predVehID,
                              display=False)    
    results.add(['DSRC',trajectoryNames[predind]]+result)

for predind in range(len(trajectory2Predictors)):
    trajectoryPredictor = trajectory2Predictors[predind]
    trajectorySettings = trajectory2Settings[predind]
    egoPredictor = ego2Predictors[predind]
    egoSettings = ego2Settings[predind]
    
    pred = []
    predVehID = []
    for simIndex in range(nsims):
        inFile = vehicleFolder+"/"+simName+"/"+np.str(simIndex+1)+".csv"
        vData = pd.read_table(inFile, sep=',')
        sensor2Data = applySensor(vData, vehicleIDtoSeek, sensor2,
                            sensor2Settings, egoSensor2, egoSensor2Settings)
                                  
        predCollision, predictVehID = applyPredictor(vData, sensor2Data,
                                                     'ego', 2.5, 3.5,
                                                     trajectoryPredictor,
                                                     trajectorySettings,
                                                     egoPredictor,
                                                     egoSettings)
        pred += [predCollision]
        predVehID += [predictVehID]
        
    result = scorePredictions(truth, truthVehID, pred, predVehID,
                              display=False)    
    results.add(['Radar',trajectoryNames[predind]]+result)
    
for predind in range(len(trajectory3Predictors)):
    trajectoryPredictor = trajectory3Predictors[predind]
    trajectorySettings = trajectory3Settings[predind]
    egoPredictor = ego3Predictors[predind]
    egoSettings = ego3Settings[predind]
    
    pred = []
    predVehID = []
    for simIndex in range(nsims):
        inFile = vehicleFolder+"/"+simName+"/"+np.str(simIndex+1)+".csv"
        vData = pd.read_table(inFile, sep=',')
        sensor3Data = applySensor(vData, vehicleIDtoSeek, sensor3,
                            sensor3Settings, egoSensor3, egoSensor3Settings)
                                  
        predCollision, predictVehID = applyPredictor(vData, sensor3Data,
                                                     'ego', 2.5, 3.5,
                                                     trajectoryPredictor,
                                                     trajectorySettings,
                                                     egoPredictor,
                                                     egoSettings)
        pred += [predCollision]
        predVehID += [predictVehID]
        
    result = scorePredictions(truth, truthVehID, pred, predVehID,
                              display=False)    
    results.add(['Combined',trajectoryNames[predind]]+result)

results.write(outputFolder+"/realOtherRandomb.csv")