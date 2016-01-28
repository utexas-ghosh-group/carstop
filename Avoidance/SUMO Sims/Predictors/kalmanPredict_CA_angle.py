# -*- coding: utf-8 -*-
"""
constant velocity Kalman filter
1/13/16
"""
# Install filterpy
# http://pythonhosted.org/filterpy/
# https://github.com/rlabbe/filterpy

import numpy as np
import pandas as pd
import sys, os
sys.path.append(os.path.realpath('/usr/local/lib/python2.7/dist-packages/filterpy'))
#from filterpy.kalman import KalmanFilter as KF
from filterpy.kalman import UnscentedKalmanFilter as UKF  
from filterpy.kalman import MerweScaledSigmaPoints as SigmaPoints
import copy

    
# State space
# x: easting
# y: northing
# v: speed
# r: angle
# a: acceleration
# w: angular velocity
# dt: time step
# state transition function
# state, noise are numpy array
def f_kal_accel(state, dt):
    x = state[0]
    y = state[1]
    v = state[2]
    r = state[3]
    a = state[4]
    w = state[5]
    
    fstate = state[:].copy()
    # integrate for position
    deltat = .001
    for ddt in np.arange(0, dt, deltat):
        fstate[0] += (v+a*ddt) * np.sin(r+w*ddt) * deltat
        fstate[1] += (v+a*ddt) * -np.cos(r+w*ddt) * deltat
    fstate[2] = v + a*dt
    fstate[3] = r + w*dt
    
    return fstate

# the difference between two states
# angle must be treated carefully, so we can't use the default
def res_x(statea,stateb):
    res = statea-stateb
    if res[3] > np.pi:
        res[3] -= 2*np.pi
    if res[3] < np.pi:
        res[3] += 2*np.pi
    return res
    
# measurement function
def h_kal_accel(state):  
    fstate = state.copy()
    return fstate[:4]     
    
# averaging function for unscented mean
def state_mean(sigmas, Wm):
                x = np.zeros(6)
                sum_sin, sum_cos = 0., 0.
                for i in range(len(sigmas)):
                    s = sigmas[i]
                    x[0] += s[0] * Wm[i]
                    x[1] += s[1] * Wm[i]
                    x[2] += s[2] * Wm[i]
                    x[4] += s[4] * Wm[i]
                    x[5] += s[5] * Wm[i]
                    sum_sin += np.sin(s[3])*Wm[i]
                    sum_cos += np.cos(s[3])*Wm[i]
                x[3] = np.arctan2(sum_sin, sum_cos)
                return x     
     
class KalmanPredict_CA_angle:
    
    def __init__(self, trueTrajectory, dt, Q=np.eye(4), R=np.eye(4)):
        n_state = len(Q)
        n_meas = len(R)
        sigmas = SigmaPoints(n_state, alpha=.5, beta=2., kappa=0.)
        ukf = UKF(dim_x=n_state, dim_z=n_meas, fx=f_kal_accel, hx=h_kal_accel,
                  dt=dt, points=sigmas, x_mean_fn = state_mean, residual_x=res_x,
                  residual_z=res_x)
        ukf.Q = Q
        ukf.R = R
        self.ukf = ukf
        self.isFirst = True
    
    def predict(self, vData, predictTimes):
        
        currState = vData.iloc[vData.shape[0]-1] # Current state (last data)
        if self.isFirst:
            self.ukf.x[0] = currState.x
            self.ukf.x[1] = currState.y
            self.ukf.x[2] = currState.speed
            self.ukf.x[3] = currState.angle
            self.ukf.x[4] = 0.
            self.ukf.x[5] = 0.
            self.isFirst = False
        else: # Not first state -> need MAIN update process
            z = np.array([currState.x,currState.y,currState.speed,currState.angle])
            # ensure that you submit the angle such that the change is smallest
            #if z[3] - self.ukf.x[3] > np.pi:
            #    z[3] = 2*np.pi - z[3]
            #if self.ukf.x[3] - z[3] > np.pi:
            #    z[3] = 2*np.pi + z[3]
            self.ukf.update(z, R=None, UT=None, hx_args=())
            
        currTime = currState['time'] # Current time
        
        # Copy temporary ukf for prediction
        returnedStates = vData[vData['time']<0] # empty state to return
        for time in predictTimes:
            ukf_temp = copy.copy(self.ukf)
            # predict with UKF
            ukf_temp.predict(dt = time-currTime, UT=None, fx_args=())
            # save predicted state
            newState = currState.copy()
            newState.x = ukf_temp.x[0]
            newState.y = ukf_temp.x[1]
            newState.speed = ukf_temp.x[2]
            newState.angle = ukf_temp.x[3]
            newState.time = time
            returnedStates = returnedStates.append(newState)
    #        print "ukf x: " + str(ukf.x[0]) + " / y: " + str(ukf.x[1])
    #        print "temp x: " + str(ukf_temp.x[0]) + " / y: " + str(ukf_temp.x[1])
            
        # MAIN prediction step
        self.ukf.predict(dt = .1, UT=None, fx_args=()) # fix dt somehow
        return returnedStates
