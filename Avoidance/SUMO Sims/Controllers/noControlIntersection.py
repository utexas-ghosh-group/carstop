# -*- coding: utf-8 -*-
"""
Follows the car in front of it (somewhat) safely, but ignores any
intersection rules.
Last change: 6/28/15
"""

import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here

class NoControlIntersection:
    speedMode = 7   #one-point difference from NoControl, but its important
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