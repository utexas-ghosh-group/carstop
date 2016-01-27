# -*- coding: utf-8 -*-
"""
Runs a predictor code and returns the output
1/27/16
"""
import numpy as np
import pandas as pd
import os
import Predictors
from applyPredictor import applyPredictor, scorePredictions

vehicleFolder = os.path.realpath("Results")
sensorFolder = os.path.realpath("Sensor Results")
outputFolder = os.path.realpath("Analysis")
paramFolder = os.path.realpath("Parameters")

nsims = 10

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
results = WriteFrame(['Type', 'communication range', 'noise magnitude',
                      'prediction algorithm', 'number of collisions',
                      'number of warnings', 'number of correct warnings',
                      'number of correct safe','early warnings','late warnings'])

options = []
for letter in ['a']:
    for commRange in [200]:
        for noiseLevel in [0., 1.]:
            for predictorType in ['kline','cline']:
                options += [[letter, commRange, noiseLevel, predictorType]]

for option in options:
    letter = option[0]
    commRange = option[1]
    noiseLevel = option[2]
    predictorType = option[3]    
    
    simName = "inter1l/"+letter
    sensorName = simName+"_"+str(int(commRange))+"_"+str(noiseLevel)+"_"
    
    print "running "+sensorName+" with "+predictorType
    
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
    if predictorType == 'kline':
        Q = np.diag([2., 2.])
        R = np.diag([2., 1.2])*noiseLevel
        trajectoryPredictor = Predictors.KalmanPredict_line
        trajectoryPredictorSettings = [ .1, 'E2W', Q, R ]
        egoPredictor = Predictors.KalmanPredict_line
        egoPredictorSettings = [ .1, 'S2N', Q, R ]
    if predictorType == 'cline':
        trajectoryPredictor = Predictors.CV_line
        trajectoryPredictorSettings = ['E2W']
        egoPredictor = Predictors.CV_line
        egoPredictorSettings = ['S2N']
    
    paramFile = paramFolder+ "/" + simName + "_param.csv"
    paramTruth = pd.read_csv(paramFile)
    
    truth = paramTruth["Collision Time"].tolist()
    truthVehID = paramTruth["Colliding Vehicle"].tolist()
    pred = []
    predVehID = []
    
    
    for simIndex in range(nsims):
        print "- nsim: " + str(simIndex+1) +" / " +str(nsims)
        vehicleFile = vehicleFolder + "/" + simName + np.str(simIndex+1) + ".csv"
        sensorFile = sensorFolder + "/" + sensorName + np.str(simIndex+1) + ".csv"
        
        vehicleData = pd.read_table(vehicleFile,sep=',') # new data read
        try:
            sensorData = pd.read_table(sensorFile,sep=",")
        except ValueError: # empty file
            pred += [-1]
            predVehID += ['']
            continue
        
        predictCollision, predictVehID = applyPredictor(vehicleData, sensorData,
                                                        'ego', 2.5, 3.5,
                                                        trajectoryPredictor,
                                                        trajectoryPredictorSettings,
                                                        egoPredictor,
                                                        egoPredictorSettings)
        pred += [predictCollision]
        predVehID += [predictVehID]
    
    result = scorePredictions(truth, truthVehID, pred, predVehID, display=False)
    
    results.add(option+result)    
    # if you want to save individual simulation's result
    #outputFile = outputFolder +"/"+sensorName+".csv"
    #paramTruth = paramTruth.iloc[:nsims]
    #paramTruth['Predicted Collision Time'] = pred
    #paramTruth['Predicted Colliding Vehicle'] = predVehID
    #paramTruth.to_csv(outputFile, sep=",")
    
results.write(outputFolder+"/inter1lAnalysis1.csv")