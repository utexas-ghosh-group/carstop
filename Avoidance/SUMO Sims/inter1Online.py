# -*- coding: utf-8 -*-
"""
1/28/16
"""
import numpy as np
import pandas as pd
from applySensor import applySensor
from applyPredictor import applyPredictor
import inter1SimOnline
import multiprocessing
import Predictors
import Sensors

nsims = 20
letters = ['a']
commRanges = [200]
noiseLevels = [0., 1.]
predictorTypes = ['cline','kline']


def getPredictorInfo(predictorType, letter, noiseLevel):
    if letter == 'a':
        egoDir = 'S2N'
        altDir = 'E2W'
    if letter == 'd':
        egoDir = 'S2E'
        altDir = 'W2E'
    
    if predictorType == 'kline':
        Q = np.diag([2., 2.])
        R = np.diag([2., 1.2]) * noiseLevel
        trajectoryPredictor = Predictors.KalmanPredict_line
        trajectoryPredictorSettings = [ .1, altDir, Q, R ]
        egoPredictor = Predictors.KalmanPredict_line
        egoPredictorSettings = [ .1, egoDir, Q, R ]
    elif predictorType == 'cline':
        trajectoryPredictor = Predictors.CV_line
        trajectoryPredictorSettings = [altDir]
        egoPredictor = Predictors.CV_line
        egoPredictorSettings = [egoDir]
        
    return [trajectoryPredictor, trajectoryPredictorSettings, egoPredictor,
            egoPredictorSettings]

options = []
for letter in letters:
    for commRange in commRanges:
        for noiseLevel in noiseLevels:
            for predictorType in predictorTypes:
                options += [[letter, commRange, noiseLevel, predictorType]]

nOptions = len(options)
XTable = np.array(options)
YTable = np.zeros((nOptions,6))

# nT, nP, nTP, nTN, early, late
def score(pred, truth, tolerance = 1.):
    if truth > 0 and pred > 0:
        if truth - pred > tolerance:
            return [1, 1, 0, 0, 1, 0]
        elif pred - truth > tolerance:
            return [1, 1, 0, 0, 0, 1]
        else:
            return [1, 1, 1, 0, 0, 0]
    elif truth > 0:
        return [1, 0, 0, 0, 0, 0]
    elif pred > 0:
        return [0, 1, 0, 0, 0, 0]
    else:
        return [0, 0, 0, 1, 0, 0]

def runProcess(queue, vehicleData, sensor, sensorSettings, egoSensor,
               egoSensorSettings, predictor, predictorSettings,
               egoPredictor, egoPredictorSettings, extraInfo):
    sensorData = applySensor(vehicleData, 'ego', sensor, sensorSettings,
                             egoSensor, egoSensorSettings)
    predTime, predID = applyPredictor(vehicleData, sensorData, 'ego', 0.5, 1.5,
                                      predictor, predictorSettings,
                                      egoPredictor, egoPredictorSettings)
    queue.put([predTime] + extraInfo)

outputQueue = multiprocessing.Queue() 
    
for sim in range(nsims):
    print str(sim)+" / "+str(nsims)
    truth, vehicleData = inter1SimOnline.init()
    
    processes = []
    for option in options:
        sensor = Sensors.DSRC
        letter = option[0]
        commRange = option[1]
        noiseLevel = option[2]
        predictorType = option[3]
        
        sensorSettings = [noiseLevel, commRange, 0.]
        egoSensor = Sensors.DSRC
        egoSensorSettings = [noiseLevel, 200., 0.]
        predictor, predictorSettings, egoPredictor, egoPredictorSettings = getPredictorInfo(
                predictorType, letter, noiseLevel)
        
        processes += [multiprocessing.Process(target = runProcess, args = (
                    outputQueue, vehicleData, sensor, sensorSettings,
                    egoSensor, egoSensorSettings, predictor, predictorSettings,
                    egoPredictor, egoPredictorSettings, option))]
        
    for p in processes:
        p.start()
    for p in processes:
        p.join()
    while not outputQueue.empty():
        result = outputQueue.get()
        location = np.all(XTable == result[1:], axis=1)
        YTable[location,:] += score(result[0], truth)

Table = pd.DataFrame(XTable, columns = ['Sim type','commrange','noise',
                                       'predictor'])
Table['nT'] = YTable[:,0]
Table['nP'] = YTable[:,1]
Table['nTP'] = YTable[:,2]
Table['nTN'] = YTable[:,3]
Table['early'] = YTable[:,4]
Table['late'] = YTable[:,5]