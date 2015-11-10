# -*- coding: utf-8 -*-
"""
Adds noise after the fact to vehicle state
Only input is uniform noise magnitude... will have to become more detailed
11/10/15
"""
import numpy as np
import pandas as pd

def GenericNoisePredict(sensedV,trueV,trueEgo,predictTimes):
    
    error = .5    # meters
    
    resultV = trueV[trueV['time']<0].copy() # empty copy
    times = trueV['time'].tolist()
    for ptime in predictTimes:
        if np.any(times==ptime):
            trueVstate = trueV[times==ptime].iloc[0].copy()
        elif max(times) < ptime: # vehicle's left the simulation...
            trueVstate = trueV.iloc[0].copy()
            trueVstate.x = 1000
            trueVstate.y = 1000
        else:
            # pick the nearest frame...
            trueVstate = trueV.iloc[np.argmin(np.abs(times-ptime))].copy()
            
            # could also try to interpolate (not complete yet)
#            beforeVtime = max(times[times<ptime])
#            beforeVstate = trueV[times==beforeVtime].iloc[0]
#            afterVtime = min(times[times>ptime])
#            afterVstate = trueV[times==afterVtime].iloc[0]
#            
#            trueVstate = beforeVstate.copy()
#            timelen = afterVtime-beforeVtime
#            trueVstate.x = beforeVstate.x*(ptime-beforeVtime)/timelen
#            trueVstate.x += afterVstate.x*(afterVtime-ptime)/timelen
#            trueVstate.y = beforeVstate.y*(ptime-beforeVtime)/timelen
#            trueVstate.y += afterVstate.y*(afterVtime-ptime)/timelen
        resultV = resultV.append(trueVstate)
    
    resultV['x'] = resultV['x'] + np.random.uniform(-error, error, resultV.shape[0])
    resultV['y'] = resultV['y'] + np.random.uniform(-error, error, resultV.shape[0])
    
    return resultV