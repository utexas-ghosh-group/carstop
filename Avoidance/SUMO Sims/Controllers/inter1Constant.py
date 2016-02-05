# -*- coding: utf-8 -*-
"""
Avoidance-less model intended for highway purposes.
It chooses a speed to reach, then stays there for a random time before moving again.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here
import random

class Inter1Constant:
    speedMode = 6   #only physics checks
    laneChangeMode = 69 # no lane change for speed purposes
    
    def __init__(self, vehID, initspeed=None, accel=None, DELTAT=0.1):
        self.ID = vehID
        self.speed = None
        self.obstacles = {}
        self.accel = None
        if accel is None:
            self.accel = random.uniform(-1,1)
        else:
            self.accel = accel
        if initspeed is None:
            self.speed = 10.
        else:
            self.speed = initspeed
        self.DELTAT = DELTAT
        
    def updateSpeed(self, speed):
        self.speed = self.speed + self.accel*self.DELTAT
        if self.speed < 0:
            self.speed = 0.
            #self.accel = 1.
    
    def nextStep(self, obstacles):
        return self.speed