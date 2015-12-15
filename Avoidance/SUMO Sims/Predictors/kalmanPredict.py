# -*- coding: utf-8 -*-
"""
designed to predict a rearEndBrake controlled vehicle
11/10/15
"""
# Install filterpy
# http://pythonhosted.org/filterpy/
# https://github.com/rlabbe/filterpy

import numpy as np
import pandas as pd
#from filterpy.kalman import KalmanFilter as KF
from filterpy.kalman import UnscentedKalmanFilter as UKF  
from filterpy.kalman import MerweScaledSigmaPoints as SigmaPoints
import copy

def initPredictor(*args):
    n_state = args[0]
    n_meas = args[1]
    dt = args[2]
    sigmas = SigmaPoints(n_state, alpha=.1, beta=2., kappa=1.)
    ukf = UKF(dim_x=n_state, dim_z=n_meas, fx=f_kal, hx=h_kal, dt=dt, points=sigmas)
    ukf.Q = 0.1*np.eye(n_state)
    ukf.R = 0.1*np.eye(n_meas)
    return ukf

# State space
# x: easting
# y: northing
# v: speed
# r: angle
# dt: time step
# state transition function
# state, noise are numpy array
def f_kal(state, dt):
    x = state[0]
    y = state[1]
    v = state[2]
    r = state[3]
    
    fstate = state[:]
    fstate[0] = x + dt*v*np.sin(r)
    fstate[1] = y + dt*v*np.cos(r)
    
    return fstate

# measurement function
def h_kal(state):  
    return state

def KalmanPredict(vData,predictTimes, *args):
    ukf = args[0]
    isFirst = args[1]
    timeNext = args[2] # One step ahead time
    
    currState = vData.iloc[vData.shape[0]-1] # Current state (last data)
    if not isFirst: # Not first state -> need MAIN update process
        z = np.array([currState.x,currState.y,currState.speed,currState.angle])
        ukf.update(z, R=None, UT=None, hx_args=())
    currTime = currState['time'] # Current time
    
    # Copy temporary ukf for prediction
    returnedStates = vData[vData['time']<0] # empty state to return
    for time in predictTimes:
        ukf_temp = copy.copy(ukf) 
        ukf_temp.predict(dt = time-currTime, UT=None, fx_args=()) # predict with UKF
        # save predicted state
        newState = currState.copy()
        newState.x = ukf_temp.x[0]
        newState.y = ukf_temp.x[1]
        newState.speed = ukf_temp.x[2]
        newState.time = time
        returnedStates = returnedStates.append(newState)
#        print "ukf x: " + str(ukf.x[0]) + " / y: " + str(ukf.x[1])
#        print "temp x: " + str(ukf_temp.x[0]) + " / y: " + str(ukf_temp.x[1])
        
    # MAIN prediction step
    ukf.predict(dt = timeNext-currTime, UT=None, fx_args=()) # predict with UKF
    return returnedStates
