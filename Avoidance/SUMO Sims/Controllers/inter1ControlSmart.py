# -*- coding: utf-8 -*-
"""
Avoidance-less model intended for highway purposes.
It chooses a speed to reach, then stays there for a random time before moving again.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
#from constants import * # sys.path is modified here
import numpy as np
import pandas as pd
import random
from Predictors import CV_line
import collisionCheck

class Inter1ControlSmart:
    speedMode = 6   #only physics checks
    laneChangeMode = 69 # no lane change for speed purposes
    
    def __init__(self, vehID, initspeed=None, maxaccel=1., DELTAT=0.1):
        self.ID = vehID
        self.speed = None
        self.obstacles = {}
        self.maxAccel = maxaccel
        if initspeed is None:
            self.speedCommand = []
        else:
            self.speedCommand = [initspeed]
        self.DELTAT = DELTAT
        self.egoPredictor = CV_line(None, 'S2N')
        self.predictor = CV_line(None, 'E2W')
        
    def updateSpeed(self, speed):
        self.speed = speed
    
    def nextStep(self, vehicles):       
        if len(self.speedCommand)==0:
            self.newSpeedCommand()
           
        mystate = None # first find own information
        for vstate in vehicles:
            if vstate.vehID == self.ID:
                mystate = vstate2df(vstate)
        if mystate is None:
            print "no find self"
            return self.speedCommand.pop()
        mystate.time = 1
        myfuture = self.egoPredictor.predict(mystate, np.arange(0.5,2.1,.1))
        
        for vstate in vehicles: # check each other vehicle for collision
            if vstate.vehID != self.ID:
                otherfuture = self.predictor.predict(vstate2df(vstate),
                                                     np.arange(0.5,2.1,.1))
                for ptime in range(myfuture.shape[0]):
                    myfutureloc = myfuture.iloc[ptime]
                    otherfutureloc = otherfuture.iloc[ptime]
                    #print 'checking'
                    #print str(myfutureloc.x)+", "+str(myfutureloc.y)
                    #print str(otherfutureloc.x)+", "+str(otherfutureloc.y)
                    if collisionCheck.check(myfutureloc, otherfutureloc):
                        print "collision detected in "+str(ptime/10.+.3)+" s"
                        self.speedCommand = ( list(np.arange(20/.447,0,-.1)) +
                                            [0.]*5 +
                                            list(np.arange(0,self.speed,.5)) )
                        self.speedCommand.pop()
                        return self.speedCommand.pop()
        return self.speedCommand.pop()
        
    def newSpeedCommand(self):
        MPH2MS = .447
        chosenSpeed = truncatedNormal(self.speed/MPH2MS, 10, 20, 50)*MPH2MS
        chosenAccel = random.uniform(0,self.maxAccel) * self.DELTAT
        meanTime = 2.0 # seconds
        chosenTime = np.floor(random.expovariate(1/meanTime) / self.DELTAT) + 1
        
        accelPeriod = np.arange(chosenSpeed, self.speed,
                                chosenAccel*np.sign(self.speed-chosenSpeed))
        if chosenTime <= len(accelPeriod):
            self.speedCommand = accelPeriod[-chosenTime:].tolist()
        else:
            steadyPeriod = [chosenSpeed]*(chosenTime-len(accelPeriod))
            self.speedCommand = steadyPeriod + accelPeriod.tolist()
        print "chosen speed " +str(chosenSpeed)
        
def vstate2df(vstate):
    return pd.DataFrame({'x':[vstate.x],'y':[vstate.y],'angle':[vstate.angle],
                         'speed':[vstate.speed],'length':[5.],'width':[2.],
                         'time':[1]})
                         
def truncatedNormal(mean, sd, minvalue = 0, maxvalue = 30):
    answer = minvalue - 1
    counter = 10000
    while (answer < minvalue or answer > maxvalue) and counter > 0:
        answer = random.gauss(mean, sd)
        counter += -1
    if counter <= 0:   # failed to sample
        return (minvalue + maxvalue) / 2
    return answer