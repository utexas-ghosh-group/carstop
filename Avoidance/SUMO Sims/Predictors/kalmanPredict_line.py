# -*- coding: utf-8 -*-
"""
constant velocity Kalman filter
2/4/16
"""
# Install filterpy
# http://pythonhosted.org/filterpy/
# https://github.com/rlabbe/filterpy

import numpy as np
import pandas as pd
import sys, os
sys.path.append(os.path.realpath('/usr/local/lib/python2.7/dist-packages/filterpy'))
import copy

def getLoc(position, route):
    if route == 'E2W':
        x = -position
        y = 101.65
        angle = -np.pi/2
    elif route == 'S2N':
        x = 101.65
        y = position
        angle = np.pi
    elif route == 'W2E':
        x = position
        y = 98.35
        angle = np.pi/2
    elif route == 'N2S':
        x = 98.35
        y = -position
        angle = 0.
        
    elif route == 'S2W':
        start = (101.65,95)
        end = (95,101.65)
        rad = np.abs(end[0]-start[0])
        if position <= start[1]:
            x = start[0]
            y = position
            angle = np.pi
        elif position >= start[1] + rad*np.pi/2:
            x = end[0] + start[1] + rad*np.pi/2 - position
            y = end[1]
            angle = -np.pi/2
        else:
            radians = (position - start[1])/rad
            y = start[1] + rad*np.sin(radians)
            x = end[0] + rad*np.cos(radians)
            angle = radians - np.pi
            
    elif route == 'E2N':
        start = (105,101.65)
        end = (101.65,105)
        rad = np.abs(end[0]-start[0])
        if position <= 200-start[0]:
            x = 200-position
            y = start[1]
            angle = -np.pi/2
        elif position >= 200-start[0] + rad*np.pi/2:
            x = end[0]
            y = end[1] + start[0] + position - 200 - rad*np.pi/2
            angle = np.pi
        else:
            radians = (position - 200+start[0])/rad
            y = end[1] - rad*np.cos(radians)
            x = start[0] - rad*np.sin(radians)
            angle = -np.pi/2 - radians
            
    elif route == 'S2E':
        start = (101.65,95)
        end = (105,98.35)
        rad = np.abs(end[0]-start[0])
        if position <= start[1]:
            x = start[0]
            y = position
            angle = np.pi
        elif position >= start[1] + np.pi/2*rad:
            x = end[0] + position - start[1] - rad*np.pi/2
            y = end[1]
            angle = np.pi/2
        else:
            radians = (position - start[1])/rad
            x = end[0] - rad*np.cos(radians)
            y = start[1] + rad*np.sin(radians)
            angle = np.pi - radians
            
    elif route == 'W2S':
        start = (95,98.35)
        end = (98.35,95)
        rad = np.abs(end[0]-start[0])
        if position <= start[0]:
            x = position
            y = start[0]
            angle = np.pi/2
        elif position >= start[0] + np.pi/2*rad:
            x = end[0]
            y = end[1] - position + start[0] + np.pi/2*rad
            angle = 0.
        else:
            radians = (position - start[0])/rad + np.pi/2
            x = start[0] + rad*np.cos(radians)
            y = start[0] + rad*np.sin(radians)
            angle = np.pi/2 - radians
            
    return [x,y,angle]

def getPos(x,y,angle,route):
    if route == 'E2W':
        position = -x
    elif route == 'S2N':
        position = y
    elif route == 'W2E':
        position = x
    elif route == 'N2S':
        position = -y
    elif route == 'S2W':
        start = (101.65,95)
        end = (95,101.65)
        center = (95,95)
        rad = np.abs(end[0]-start[0])
        if y <= start[1]:
            position = y
        elif x <= end[0]:
            position = start[1] + rad*np.pi/2 + (end[0]-x)
        else:
            position = start[1] + np.arctan2(y-center[1],x-center[0])*rad
    elif route == 'E2N':
        start = (105,101.65)
        end = (101.65,105)
        center = (105,105)
        rad = np.abs(end[0]-start[0])
        if x >= start[0]:
            position = 200 - x
        elif y >= end[1]:
            position = 200 - start[0] + rad*np.pi/2 + (y-end[1])
        else:
            angle = -np.arctan2(y-center[1],x-center[0]) - np.pi/2
            position = 200-start[0] + angle*rad
    elif route == 'S2E':
        start = (101.65,95)
        end = (105,98.35)
        center = (105,95)
        rad = np.abs(end[0]-start[0])
        if y <= start[1]:
            position = y
        elif x >= end[0]:
            position = start[1] + rad*np.pi/2 + (x-end[0])
        else:
            angle = -np.arctan2(y-center[1],x-center[0]) + np.pi
            position = start[1] + angle*rad
    elif route == 'W2S':
        start = (95,98.35)
        end = (98.35,95)
        center = (95,95)
        rad = np.abs(end[0]-start[0])
        if x <= start[0]:
            position = x
        elif y <= end[1]:
            position = start[0] + rad*np.pi/2 + (end[1]-y)
        else:
            angle = -np.arctan2(y-center[1],x-center[0]) + np.pi/2
            position = start[0] + angle*rad
    return position

# State space
# x: easting
# v: speed
# a: acceleration
# dt: time step
# state transition function
# state, noise are numpy array
def f_kal_a(state, dt):
    x = state[0]
    v = state[1]
    a = state[2]
    
    fstate = state[:].copy()
    al = 3.5
    if dt < al:
        fstate[0] = x + dt*v + dt*dt*a/2. - a*np.exp(dt-al) + a*np.exp(al)*(1+dt)
        fstate[1] = v + dt*a - a*np.exp(dt-al) + a*np.exp(al)
    else:
        fstate[0] = x + dt*v + al*al*a/2. - a + a*np.exp(al)*(1+al)
        fstate[1] = v + al*a - a + a*np.exp(al)
    
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


class KalmanPredict_line:    
    
    def __init__(self, trueTrajectory, dt, route, Q=np.eye(2), R=np.eye(2)):
        
        #from filterpy.kalman import KalmanFilter as KF
        from filterpy.kalman import UnscentedKalmanFilter as UKF  
        from filterpy.kalman import MerweScaledSigmaPoints as SigmaPoints        
        
        n_state = len(Q)
        n_meas = len(R)
        sigmas = SigmaPoints(n_state, alpha=.1, beta=2., kappa=0.)
        ukf = UKF(dim_x=n_state, dim_z=n_meas, fx=f_kal_v, hx=h_kal,
                  dt=dt, points=sigmas)
        ukf.Q = Q
        ukf.R = R
        self.ukf = ukf
        self.isFirst = True
        self.route = route
    
    def predict(self, vData, predictTimes):
        
        currState = vData.iloc[vData.shape[0]-1] # Current state (last data)
        position = getPos(currState.x,currState.y,currState.angle,self.route)
        if self.isFirst:
            self.ukf.x[0] = position
            self.ukf.x[1] = currState.speed
            self.isFirst = False
        else: # Not first state -> need MAIN update process
            z = np.array([position,currState.speed])
            self.ukf.update(z, R=None, UT=None, hx_args=())
            
        currTime = currState['time'] # Current time
        
        # Copy temporary ukf for prediction
        returnedStates = vData[vData['time']<0] # empty state to return
        for time in predictTimes:
            newState = currState.copy()
            
            # predict with UKF
            ukf_temp = copy.copy(self.ukf)            
            ukf_temp.predict(dt = time-currTime, UT=None, fx_args=())
            new_x,new_y,new_angle = getLoc(ukf_temp.x[0], self.route)
            newState.x = new_x
            newState.y = new_y
            newState.speed = ukf_temp.x[1]
            newState.angle = new_angle
            newState.time = time
            
            # use expected value instead - this is f(E[X]), instead of E[f(X)]
            # but it seems to be working better
#            newArray = f_kal(self.ukf.x, time-currTime)
#            newState.x = newArray[0]    
#            newState.y = newArray[1]   
#            newState.speed = newArray[2]   
#            newState.angle = newArray[3]
#            newState.time = time
            
            returnedStates = returnedStates.append(newState)
            
        # MAIN prediction step
        self.ukf.predict(dt = .1, UT=None, fx_args=()) # fix dt somehow
        return returnedStates 