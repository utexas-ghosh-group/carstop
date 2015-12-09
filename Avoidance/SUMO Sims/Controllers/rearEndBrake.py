# -*- coding: utf-8 -*-
"""
Avoidance-less model intended for highway purposes.
It chooses a speed to reach, then stays there for a random time before moving again.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
#from constants import * # sys.path is modified here
import numpy as np

class RearEndBrake:
    
    def __init__(self, speed):
        self.speed = speed
        self._brakeTime = np.random.exponential(30.0) + 10 #1000m at 25 m/s
        self._brakeMagnitude = np.random.uniform(1.0,7.0)
        self.time = 0
        
    def update(self, DELTAT):
        self.time = self.time + DELTAT
        if self.time > self._brakeTime:
            newspeed = self.speed - self._brakeMagnitude*DELTAT
        else:
            newspeed = self.speed
        newspeed = max(newspeed,0.0)
        self.speed = newspeed
    
    def nextStep(self):
        return self.speed