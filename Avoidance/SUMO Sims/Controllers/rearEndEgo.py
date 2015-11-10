# -*- coding: utf-8 -*-
"""
remains at the speed at which it was created
11/9/15
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here

class RearEndEgo:
    speedMode = 6   #only physics checks
    laneChangeMode = 69
    
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
        return self.speed