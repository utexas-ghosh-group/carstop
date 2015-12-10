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
    
    def __init__(self, speed, accel=0.):
        self.speed = speed
        self.accel = accel
        self.brakeTime = np.random.exponential(30.0) + 5. #1000m at 25 m/s
        self.brakeMagnitude = np.random.uniform(2.0,9.0)
        self.time = 0
        
    def update(self, DELTAT):
        self.time = self.time + DELTAT
        if self.time > self.brakeTime:
            newspeed = self.speed - self.brakeMagnitude*DELTAT
        else:
            newspeed = self.speed + self.accel*DELTAT
        newspeed = max(newspeed,0.0)
        self.speed = newspeed
    
    def nextStep(self):
        return self.speed