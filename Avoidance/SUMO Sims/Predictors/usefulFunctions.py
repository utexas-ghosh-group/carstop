# -*- coding: utf-8 -*-
"""
contains methods that will be used by multiple predictors
"""
import numpy as np

# one-dimensional acceleration model
# uses a damped acceleration, a*(1 - e^cutoff)
def CA_physics(dt, x, v, a=0, cutoff=1000, preventNegativeSpeed=True):
    dt = np.array(dt)*1.
    returnSingle = dt.ndim == 0
    if returnSingle:
        dt = np.array([dt])
    
    t1 = np.minimum([cutoff]*len(dt), np.array(dt))
    t2 = np.array(dt)-t1
    expoff = np.exp(-cutoff)
    
    v1 = v + a*t1 - a*expoff*(np.exp(t1) - 1)
    
    if preventNegativeSpeed:
        if v < 0:
            #print "CA_physics called with v < 0"
            v1 = np.minimum(v1, v1*0.)
            #return None
        negativePoints = v1 < 0
        v1[negativePoints] = 0
        if a >= 0:
            t0 = None
        elif cutoff > 5:
            t0 = -v/a # find time where v hits 0
        else: # second-order polynomial approx.
            t0 = 1/expoff - 1 + np.power(
                            np.power(1/expoff-1,2) + 2*v/a/expoff, .5)
        t1[negativePoints] = t0
        #x1 = x + v*t0 + a*np.power(t0,2)/2 - a*expoff*(np.exp(t0) - t0 - 1)
        
    x1 = x + v*t1 + a*np.power(t1,2)/2 - a*expoff*(np.exp(t1) - t1 - 1)
        
    # continue after the cutoff has been passed
    v2 = v1
    x2 = x1 + v1*t2
    
    if returnSingle:
        return [x2[0], v2[0]]
    return [x2, v2]