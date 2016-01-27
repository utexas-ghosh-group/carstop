# -*- coding: utf-8 -*-
"""
constant velocity model, plus CA-style models
1/18/16
"""
# Install filterpy
# http://pythonhosted.org/filterpy/
# https://github.com/rlabbe/filterpy

import numpy as np
import pandas as pd
from usefulFunctions import CA_physics

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
        rad = start[0]-end[0]
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
        rad = start[0]-end[0]
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
        rad = end[0]-start[0]
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
        rad = end[0]-start[0]
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
        rad = np.abs(start[0]-center[0])
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
        rad = np.abs(start[0]-center[0])
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
        rad = np.abs(start[0]-center[0])
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
        rad = np.abs(start[0]-center[0])
        if x <= start[0]:
            position = x
        elif y <= end[1]:
            position = start[0] + rad*np.pi/2 + (end[1]-y)
        else:
            angle = -np.arctan2(y-center[1],x-center[0]) + np.pi/2
            position = start[0] + angle*rad
    return position


class CV_line:
    
    def __init__(self, trueTrajectory, route):
        self.route = route
        self.lastSpeed = None
    
    def predict(self, vData, predictTimes):
        
        currState = vData.iloc[vData.shape[0]-1] # Current state (last data)
        position = getPos(currState.x,currState.y,currState.angle,self.route)
        
        if self.lastSpeed is None:
            accel = 0
        else:
            accel = currState.speed - self.lastSpeed
            self.lastSpeed = currState.speed
        
        returnedStates = vData[vData['time']<0] # empty state to return
        for time in predictTimes:
            newState = currState.copy()
            newPos, newSpeed = CA_physics(time-currState.time, position,
                                          currState.speed) # no accel right now
            new_x,new_y,new_angle = getLoc(newPos, self.route)
            newState.x = new_x
            newState.y = new_y
            newState.speed = newSpeed
            newState.angle = new_angle
            newState.time = time
            
            returnedStates = returnedStates.append(newState)
            
        return returnedStates 