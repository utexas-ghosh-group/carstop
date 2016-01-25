# -*- coding: utf-8 -*-
"""
Avoidance-less model intended for highway purposes.
It chooses a speed to reach, then stays there for a random time before moving again.
1/18/16
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here
import numpy as np
import random

class RearEndRandom:
    
    def __init__(self, initspeed=None, maxaccel=1., maxdecel=5., DELTAT=0.1):
        self.speed = initspeed
        self.obstacles = {}
        self.maxAccel = maxaccel
        self.maxDecel = maxdecel
        if initspeed is None:
            self.speedCommand = []
        else:
            self.speedCommand = [initspeed]
        self.DELTAT = DELTAT
        
    def update(self, DELTAT):
        if len(self.speedCommand)==0:
            self.newSpeedCommand()
        self.speed = self.speedCommand.pop()
    
    def nextStep(self):        
        return self.speed
        
    def newSpeedCommand(self):
        MPH2MS = .447
        chosenSpeed = truncatedNormal(self.speed/MPH2MS,10,30,85)*MPH2MS
        if chosenSpeed > self.speed:
            chosenAccel = random.uniform(0,self.maxAccel) * self.DELTAT
        else:
            chosenAccel = random.uniform(-self.maxDecel,0) * self.DELTAT
        meanTime = 3.0 # seconds
        chosenTime = np.floor(random.expovariate(1/meanTime) / self.DELTAT) + 1
        
        accelPeriod = np.arange(chosenSpeed, self.speed, chosenAccel)
        if chosenTime <= len(accelPeriod):
            self.speedCommand = accelPeriod[-chosenTime:].tolist()
        else:
            steadyPeriod = [chosenSpeed]*(chosenTime-len(accelPeriod))
            self.speedCommand = steadyPeriod + accelPeriod.tolist()
        
def truncatedNormal(mean, sd, minvalue = 0, maxvalue = 30):
    answer = minvalue - 1
    counter = 5000
    while (answer < minvalue or answer > maxvalue) and counter > 0:
        answer = random.gauss(mean, sd)
        counter += -1
    if counter <= 0:   # failed to sample
        print "truncated normal failed"
        return (minvalue + maxvalue) / 2
    return answer