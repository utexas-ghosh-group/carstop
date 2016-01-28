# -*- coding: utf-8 -*-
"""
designed to predict a rearEndBrake controlled vehicle
1/19/16
"""
import numpy as np
import pandas as pd
from usefulFunctions import CA_physics
   
def f_kal_a(state, dt):
    x = state[0]
    v = state[1]
    a = state[2]
    
    fstate = state[:].copy()
    newx, newv = CA_physics(dt,x,v,a,cutoff = 5., preventNegativeSpeed = False)
    fstate[0] = newx
    fstate[1] = newv
    
    return fstate

def f_kal_v(state, dt):
    x = state[0]
    v = state[1]
    
    fstate = state[:].copy()
    fstate[0] = x + dt*v
    return fstate

# measurement function
def h_kal(state):  
    return state.copy()[:2]   

def movePhysics(vstate, accel, predictTime):
    newstate = vstate.copy()
    dT = predictTime - vstate.time
    newx,newv = CA_physics(dT, vstate.x, vstate.speed, accel, cutoff = 5.)
    newstate.speed = newv
    newstate.x = newx
    newstate.y = -8.25
    newstate.time = predictTime
    return newstate

class RearEndKalman:
    def __init__(self, trueTrajectory, dt=.1, noise = 0.):
        from filterpy.kalman import UnscentedKalmanFilter as UKF  
        from filterpy.kalman import MerweScaledSigmaPoints as SigmaPoints
        self.dt = dt
        sigmas = SigmaPoints(3, alpha=.1, beta=2., kappa=0.)
        self.KF = UKF(dim_x=3, dim_z=2, fx=f_kal_a, hx=h_kal,
                  dt=dt, points=sigmas)
        self.KF.Q = np.diag([1., 0.5, 0.2])
        self.KF.R = np.diag([2., 1.12])*noise + np.diag([.05, .05])
        self.first = True
            
        
    def predict(self, vData, predictTimes):
        nobs = vData.shape[0]        
        vcurrentState = vData.iloc[nobs-1].copy()
        
        # first check if vehicle has already stopped, don't need KF then     
        if np.mean(vData['speed'].iloc[nobs-2:nobs]) <= 0:
            returnedStates = vData[vData['time']<0] # empty
            for time in predictTimes:
                vNextState = vcurrentState.copy()
                vNextState.time = time
                vNextState.y = -8.25
                returnedStates = returnedStates.append(vNextState)
            return returnedStates
        
#        # train KF
#        self.KF.x[0] = vData.iloc[0].x
#        self.KF.x[1] = vData.iloc[0].speed
#        self.KF.predict()
#        
#        for time in np.arange(1,nobs-1):
#            vState = vData.iloc[time]
#            self.KF.update(np.array([vState.x,vState.speed]))
#            self.KF.predict()
#        self.KF.update(np.array([vData.iloc[nobs-1].x,vData.iloc[nobs-1].speed]))
        
        vState = vData.iloc[nobs-1]
        if self.first:
            self.KF.x[0] = vState.x
            self.KF.x[1] = vState.speed
            #self.KF.predict()
            self.first = False
        else:
            if vState.speed < 0:
                self.KF.update(np.array([vState.x, 0.]))
            else:
                self.KF.update(np.array([vState.x,vState.speed]))        
        
        # now you can predict
        # return a dataframe of answers
        vcurrentState = vData.iloc[vData.shape[0]-1].copy()
        vcurrentState.x = self.KF.x[0]
        vcurrentState.speed = self.KF.x[1]
        returnedStates = vData[vData['time']<0] # empty
        for time in predictTimes:
            vNextState = movePhysics(vcurrentState, self.KF.x[2]/self.dt, time)
            returnedStates = returnedStates.append(vNextState)
            
        self.KF.predict()
        return returnedStates