#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Vehicles following set paths along a road, with a constant velocity model.

Longer prediction than the first simulation.
"""

import numpy as np
import alarms, motionModels, scoring
from collections import defaultdict
from regressor_2 import Model

## the number of simulations to perform
nsims = 300
## how often to utilize the motion model and to check for collision. (s)
timeres = .1
## the time duration of each simulation, and of the alarms' predictions. (s)
timelen = 2.5
## Each vehicle is started within a certain coordinate range, then moved
## backwards for 'timeback' seconds. Because the forward motion is partially
## random, this does not constrain the vehicles' positions to be within this
## coordinate range.
timeback = 2.
## The uncertainty in each vehicle's initial state is assumed to be Gaussian
## with this covariance matrix.
initialcovariance = np.diag([2., .5]) * .1
## name with which to save plots and tables; None = display but don't save
savename = 'sim2' # None #

times = np.array([timeres] * int(timelen/timeres))
MM1 = motionModels.MM_LineCV('left-right', np.diag([2., .5]))
MM2 = motionModels.MM_LineCV('right-down', np.diag([2., .5]))

v1 = np.random.normal(10., 5., nsims)
x1 = np.random.normal(5, 2., nsims) - timeback * v1
vehicle1 = np.array((x1,v1)).T
v2 = np.random.normal(10., 5., nsims)
x2 = np.random.normal(-5, 2., nsims) - timeback * v2
vehicle2 = np.array((x2,v2)).T


mc_samplecounts = np.logspace(1,3,3).astype(int)
model = Model('MLP')
        
print "running"
    
truth = []
results = defaultdict(list)
for sim in range(nsims):
    veh1 = vehicle1[sim,:]
    veh2 = vehicle2[sim,:]
    veh1 = motionModels.initialState_normal(veh1, initialcovariance)
    veh2 = motionModels.initialState_normal(veh2, initialcovariance)
    
    # find 'true' collision occurrences by using very high-res MCS alarm
    pred, rt = alarms.alarm_truth(veh1, veh2, MM1, MM2, np.array(times))
    truth += [pred]
    results['optimal'] += [(pred, rt)]
    
    for nSamples in mc_samplecounts:
        result = alarms.alarm_MCS(veh1, veh2, MM1, MM2, times, nSamples)
        label = str(nSamples)+" MCS"
        results[label] += [result]
            
    result = model.alarm(veh1, veh2, MM1, MM2, times)
    results['MLP'] += [result]
    
    result = alarms.alarm_expected(veh1, veh2, MM1, MM2, times)
    results['expected'] += [result]

    result = alarms.alarm_UT_1(veh1, veh2, MM1, MM2, times)
    results['UT 1'] += [result]
    
    result = alarms.alarm_UT_2(veh1, veh2, MM1, MM2, times)
    results['UT 2'] += [result]


truth = np.array(truth)
ProbabilityOfCollision = sum(truth)/nsims
print "collisions "+str(ProbabilityOfCollision)

scoring.plotROC(truth, results, savename)
bigTable = scoring.bigTable(truth, results, z_cost_vals = [1,10,100])
print bigTable
if not savename is None:
    bigTable.to_csv(savename+'_scores.csv')