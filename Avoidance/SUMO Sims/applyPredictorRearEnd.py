# -*- coding: utf-8 -*-
"""
Runs a predictor code and returns the output
Has the option to run in parallel (batch size > 1)
2/8/16
"""
import numpy as np
import pandas as pd
import os
import Predictors
import multiprocessing
from applyPredictor import applyPredictor, scorePredictions

vehicleFolder = os.path.realpath("Results")
sensorFolder = os.path.realpath("Sensor Results")
outputFolder = os.path.realpath("Analysis")
paramFolder = os.path.realpath("Parameters")

nsims = 30
batch_size = 5

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
results = WriteFrame(['Type', 'noise magnitude',
                      'prediction algorithm', 'number of collisions',
                      'number of warnings', 'number of correct warnings',
                      'number of correct safe','early warnings','late warnings'])

options = []
for moveType in ['rtest']:
    for noiseLevel in [0., 1.]:
        for predictorType in ['movingaverage','Kalman']:
            options += [[moveType, noiseLevel, predictorType]]

def runProcess(queue, vehicleData, sensorData, trajPredictor,
                trajPredictorSettings, egoPredictor, egoPredictorSettings, truth):
    predTime, predID = applyPredictor(vehicleData, sensorData, 'ego', 2.5, 3.5,
                                      trajPredictor, trajPredictorSettings,
                                      egoPredictor, egoPredictorSettings)
    queue.put([predTime, predID, truth])

for option in options:
    moveType = option[0]
    noiseLevel = option[1]
    predictorType = option[2]    
    
    simName = "rearEnd/" + moveType
    sensorName = simName + "_" + str(noiseLevel)
    
    print "running "+sensorName+" with "+predictorType
    
    if predictorType == 'Kalman':
        trajectoryPredictor = Predictors.RearEndKalman
        trajectoryPredictorSettings = [ .1, noiseLevel ]
        egoPredictor = Predictors.RearEndKalman
        egoPredictorSettings = [ .1, noiseLevel ]
    if predictorType == 'movingaverage':
        trajectoryPredictor = Predictors.RearEndCA
        trajectoryPredictorSettings = [.1]
        egoPredictor = Predictors.RearEndCA
        egoPredictorSettings = [.1]
    
    paramFile = paramFolder+ "/" + simName + "_param.csv"
    paramTruth = pd.read_csv(paramFile)
    
    truth = paramTruth["Collision Time"].tolist()
    #truthVehID = paramTruth["Colliding Vehicle"].tolist()
    truthVehID = ['lead']*len(truth)
    pred = []
    predVehID = []
    
    outputQueue = multiprocessing.Queue()    
    
    if batch_size == 1: # run one file at a time
        for simIndex in range(nsims):
            print "- nsim: " + str(simIndex+1) +" / " +str(nsims)
            vehicleFile = vehicleFolder + "/" + simName + "/" +\
                            np.str(simIndex+1) + ".csv"
            sensorFile = sensorFolder + "/" + sensorName + "/" +\
                            np.str(simIndex+1) + ".csv"
            
            vehicleData = pd.read_table(vehicleFile,sep=',') # new data read
            try:
                sensorData = pd.read_table(sensorFile,sep=",")
            except ValueError: # empty file
                pred += [-1]
                predVehID += ['']
                continue
                
            predCollision, predVehid = applyPredictor(vehicleData, sensorData,
                                                      'ego', 2.5, 3.5,
                                                      trajectoryPredictor,
                                                      trajectoryPredictorSettings,
                                                      egoPredictor,
                                                      egoPredictorSettings)
            pred += [predCollision]
            predVehID += [predVehid]
        thistruth = truth
    else:
        thistruth = []
        for batch in np.arange(0,nsims,batch_size):
            print "- batch " + str(batch) +" / " +str(nsims)
            processes = []
            
            for simIndex in range(batch, min(batch+batch_size, nsims)):
                
                vehicleFile = vehicleFolder + "/" + simName + "/" +\
                                    np.str(simIndex+1) + ".csv"
                sensorFile = sensorFolder + "/" + sensorName + "/" +\
                                    np.str(simIndex+1) + ".csv"
                
                vehicleData = pd.read_table(vehicleFile,sep=',') # new data read
                try:
                    sensorData = pd.read_table(sensorFile,sep=",")
                except ValueError: # empty file
                    pred += [-1]
                    predVehID += ['']
                    continue
            
                processes += [multiprocessing.Process(target=runProcess, args=(
                            outputQueue, vehicleData, sensorData,
                            trajectoryPredictor, trajectoryPredictorSettings,
                            egoPredictor, egoPredictorSettings,
                            truth[simIndex]))]
                            
            for p in processes:
                p.start()
            for p in processes:
                p.join()
            while not outputQueue.empty():
                predictedCollision, predictedVehID, truval = outputQueue.get()
                pred += [predictedCollision]
                predVehID += [predictedVehID]
                thistruth += [truval]
            
    result = scorePredictions(thistruth, truthVehID, pred, predVehID,
                                                              display=True)
    
    results.add(option+result)    
    # if you want to save individual simulation's result
    #outputFile = outputFolder +"/"+sensorName+".csv"
    #paramTruth = paramTruth.iloc[:nsims]
    #paramTruth['Predicted Collision Time'] = pred
    #paramTruth['Predicted Colliding Vehicle'] = predVehID
    #paramTruth.to_csv(outputFile, sep=",")
    
results.write(outputFolder+"/inter1lrear1.csv")