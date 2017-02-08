# -*- coding: utf-8 -*-
"""
Simulations for vehicles following the bicycle model.

Format is very similar to simulate_1, which has more documentation.
"""

import numpy as np
import alarms, motionModels, scoring
from collections import defaultdict
from regressor_3 import Model

nsims = 1000
timeres = .1
timelen = 1.
timeback = 0.5
initialnoise = np.diag([1., 1., .5, .1, 0.05, 0.01]) * .1
savename = 'sim3'


times = np.zeros((int(timelen/timeres),)) + timeres
noise = np.diag([1., 1., .5, .1, 0.05, 0.01])
MM1 = motionModels.MM_Bicycle(noise)
MM2 = motionModels.MM_Bicycle(noise)

a = np.random.normal(0., 1., nsims)
w = np.random.uniform(-.5, .5, nsims)
v = np.random.normal(10., 5., nsims) - timeback * a
th = np.random.uniform(-np.pi, np.pi, nsims)
x = np.random.normal(0., 5., nsims) - timeback * v * np.cos(th)
y = np.random.normal(0., 5., nsims) - timeback * v * np.sin(th)
vehicle1 = np.array((x,y,th,v,a,w)).T
a = np.random.normal(0., 1., nsims)
w = np.random.uniform(-.5, .5, nsims)
v = np.random.normal(10., 5., nsims) - timeback * a
th = np.random.uniform(-np.pi, np.pi, nsims)
x = np.random.normal(0., 5., nsims) - timeback * v * np.cos(th)
y = np.random.normal(0., 5., nsims) - timeback * v * np.sin(th)
vehicle2 = np.array((x,y,th,v,a,w)).T

mc_samplecounts = np.logspace(1,4,4).astype(int)
model = Model('MLP')

    
truth = []
results = defaultdict(list)
for sim in range(nsims):
    veh1 = vehicle1[sim,:]
    veh2 = vehicle2[sim,:]
    veh1 = motionModels.initialState_normal(veh1, initialnoise)
    veh2 = motionModels.initialState_normal(veh2, initialnoise)
    
    # find 'real' collision occurrences by using very high-res particle alarm
    pred, rt = alarms.alarm_truth(veh1, veh2, MM1, MM2, times)
    truth += [pred]
    results['optimal'] += [(pred, rt)]
    
    # estimators
    for nSamples in mc_samplecounts:
        result = alarms.alarm_MCS(veh1, veh2, MM1, MM2, times, nSamples)
        label = str(nSamples)+" MCS"
        results[label] += [result]
        
    result = alarms.alarm_UT_1(veh1, veh2, MM1, MM2, times)
    results['UT 1'] += [result]
    
    result = alarms.alarm_UT_2(veh1, veh2, MM1, MM2, times)
    results['UT 2'] += [result]
    
    result = alarms.alarm_expected(veh1, veh2, MM1, MM2, times)
    results['expected'] += [result]
    
    result = model.alarm_model(veh1, veh2, MM1, MM2, times)
    results['MLP'] += [result]
    
    
truth = np.array(truth)
ProbabilityOfCollision = sum(truth)/nsims
print "collisions "+str(ProbabilityOfCollision)

scoring.plotROC(truth, results, savename)
bigTable = scoring.bigTable(truth, results, z_cost_vals = [1,10,100])
print bigTable
if not savename is None:
    bigTable.to_csv(savename+'_scores.csv')