# -*- coding: utf-8 -*-
"""
Contains methods that will be used by many Sensor types.
Last modified 6/20/15
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Sensors")])
from constants import *
from math import cos, sin, atan2

# realign like in collisionCheck.py
def realignV(refstate, vstate):
    tempx = vstate.x - refstate.x
    tempy = vstate.y - refstate.y
    newAngle = vstate.angle - refstate.angle
    relativeDist = pow((pow(tempx,2) + pow(tempy,2)),.5)
    relativeAngle = atan2(tempy,tempx)
    rotationAngle = relativeAngle - refstate.angle
    
    realignedState = vstate.copy()
    realignedState.x = relativeDist * cos(rotationAngle)
    realignedState.y = relativeDist * sin(rotationAngle)
    realignedState.angle = newAngle
    return realignedState
    
# returns the minimum distance of this car
# w.r.t the front&center of the reference car
def distance(refstate, vstate):
    tempx = vstate.x - refstate.x
    tempy = vstate.y - refstate.y
    # for now, be lazy and do this
    return pow( pow(tempx,2.0) + pow(tempy,2.0), .5)