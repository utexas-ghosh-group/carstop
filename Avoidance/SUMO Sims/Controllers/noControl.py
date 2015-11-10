# -*- coding: utf-8 -*-
"""
Simple example of collision-avoidance model.  Stores all permanent
characteristics upon construction, and takes its speed and the position of other objects
(relative to this one) every step.  Returns the speed for the next turn.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here

class NoControl:
    speedMode = 6   #only physics checks
    laneChangeMode = 85
    
    def __init__(self, vehID, vParams = None, DELTAT=1.0):
        self.ID = vehID
        self.speed = None
        self.obstacles = {}
        if vParams == None:
            self.maxAccel = 1
        else:
            self.maxAccel = vParams[2]
        
    def updateSpeed(self, speed):
        self.speed = speed
    
    def nextStep(self, obstacles):
        return self.speed + self.maxAccel