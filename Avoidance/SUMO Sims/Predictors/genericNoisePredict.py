# -*- coding: utf-8 -*-
"""
Adds noise after the fact to vehicle state
Only input is uniform noise magnitude... will have to become more detailed
11/10/15
"""
import numpy as np
import pandas as pd

class GenericNoisePredict:
    
    def __init__(self, trueTrajectory, error = 0.):
        self.trueV = trueTrajectory
        self.error = error
    
    def predict(self, vData, predictTimes):
    
        resultV = self.trueV[self.trueV['time']<0].copy() # empty copy
        
        nullV = self.trueV.iloc[0].copy() # far away enough so it won't matter
        nullV.x = 1000
        nullV.y = 1000      
        
        times = self.trueV['time'].tolist()
        for ptime in predictTimes:
            if np.any(times==ptime):
                trueVstate = self.trueV[times==ptime].iloc[0].copy()
            elif max(times) < ptime or vData.shape[0] == 0:
                # vehicle's left the simulation... or has not been detected yet
                trueVstate = nullV.copy()
            else:
                # pick the nearest frame...
                nearestTime = np.argmin( np.abs( times-ptime ) )
                trueVstate = self.trueV.iloc[nearestTime].copy()
                
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
        
        resultV['x'] = resultV['x'] + np.random.uniform(-self.error,
                                                self.error, resultV.shape[0])
        resultV['y'] = resultV['y'] + np.random.uniform(-self.error,
                                                self.error, resultV.shape[0])        
        return resultV