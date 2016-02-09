# -*- coding: utf-8 -*-
"""
Runs a predictor code and returns the output
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

nsims = 10
batch_size = 5

def runProcess(queue, vehicleData, sensorData, trajPredictor,
                trajPredictorSettings, egoPredictor, egoPredictorSettings, truth):
    predTime, predID = applyPredictor(vehicleData, sensorData, 'ego', 2.5, 3.5,
                                      trajPredictor, trajPredictorSettings,
                                      egoPredictor, egoPredictorSettings)
    queue.put([predTime, predID, truth])

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
results = WriteFrame(['Type', 'communication range', 'noise magnitude','sensor angle',
                      'prediction algorithm', 'number of collisions',
                      'number of warnings', 'number of correct warnings',
                      'number of correct safe','early warnings','late warnings'])

options = []
for letter in ['b']:
    for commRange in [60., 150.]:
        for noiseLevel in [0., .5, 1.]:
            for angle in [.125, .25, .5]:
                for predictorType in ['caline','cvline','kline']:
                    options += [[letter, commRange, noiseLevel, angle, predictorType]]

for option in options:
    letter, commRange, noiseLevel, angle, predictorType = option   
    
    simName = "inter1l/constant_"+letter
    sensorName = simName+"_"+str(int(commRange))+"_"+str(noiseLevel)+"_"+str(angle)
    
    print "running "+sensorName+" with "+predictorType
    
    if predictorType == 'kline':
        Q = np.diag([2., 2.])
        R = np.diag([2., 1.2])*noiseLevel
        trajectoryPredictor = Predictors.KalmanPredict_line
        trajectoryPredictorSettings = [ .1, 'E2N', Q, R ]
        egoPredictor = Predictors.KalmanPredict_line
        egoPredictorSettings = [ .1, 'S2N', Q, R ]
    if predictorType == 'cvline':
        trajectoryPredictor = Predictors.CV_line
        trajectoryPredictorSettings = ['E2N']
        egoPredictor = Predictors.CV_line
        egoPredictorSettings = ['S2N']
    if predictorType == 'caline':
        trajectoryPredictor = Predictors.CV_line
        trajectoryPredictorSettings = ['E2N', 3.5]
        egoPredictor = Predictors.CV_line
        egoPredictorSettings = ['S2N', 3.5]
    
    paramFile = paramFolder+ "/" + simName + "_param.csv"
    paramTruth = pd.read_csv(paramFile)
    
    truth = paramTruth["Collision Time"].tolist()
    truthVehID = paramTruth["Colliding Vehicle"].tolist()
    pred = []
    predVehID = []
    
    if batch_size == 1: # run one file at a time
        for simIndex in range(nsims):
            #print "- nsim: " + str(simIndex+1) +" / " +str(nsims)
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
            
            predCollision, predictVehID = applyPredictor(vehicleData, sensorData,
                                                         'ego', 2.5, 3.5,
                                                         trajectoryPredictor,
                                                         trajectoryPredictorSettings,
                                                         egoPredictor,
                                                         egoPredictorSettings)
            pred += [predCollision]
            predVehID += [predictVehID]
        thistruth = truth
    else:
        outputQueue = multiprocessing.Queue()
        thistruth = []
        for batch in np.arange(0,nsims,batch_size):
            #print "- batch " + str(batch) +" / " +str(nsims)
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
                              display=False)
    
    results.add(option+result)    
    # if you want to save individual simulation's result
#    outputFile = outputFolder +"/"+sensorName+".csv"
#    paramTruth = paramTruth.iloc[:nsims]
#    paramTruth['Predicted Collision Time'] = pred
#    paramTruth['Predicted Colliding Vehicle'] = predVehID
#    paramTruth.to_csv(outputFile, sep=",")
    
results.write(outputFolder+"/herpinter1lConstantc.csv")
