# -*- coding: utf-8 -*-
"""
Avoidance-less model intended for highway purposes.
It chooses a speed to reach, then stays there for a random time before moving again.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here
import numpy as np

class RearEndBrake:
    speedMode = 6   #only physics checks
    laneChangeMode = 69 # no lane change for speed purposes
    
    def __init__(self, vehID, vParams = None, DELTAT=1.0):
        self.ID = vehID
        self.speed = None
        self.obstacles = {}
        if vParams == None:
            self.maxAccel = 1
        else:
            self.maxAccel = vParams[2]
        self.brakeTime = np.random.exponential(40.0) #1000m at 25 m/s
        self.brakeMagnitude = np.random.uniform(1.0,7.0)
        self.time = 0
        self.DELTAT = DELTAT
        
    def updateSpeed(self, speed):
        self.speed = speed
        self.time = self.time + self.DELTAT
    
    def nextStep(self, obstacles):        
        if self.time > self.brakeTime:
            newspeed = self.speed - self.brakeMagnitude*self.DELTAT
        else:
            newspeed = self.speed
        return  max(newspeed,0.0)