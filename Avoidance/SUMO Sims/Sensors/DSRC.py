#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gives a vehicle approximate knowledge about others, if they are close enough.
Note that it realigns them as if this vehicle has
position (0,0) and angle 0.
Last modified 11/09/15
"""
import sys, os
sys.path.append(os.path.realpath(__file__)[:-len("/Sensors/DSRC.py")])
from constants import *
from usefulMethods import realignV, distance
import random

class DSRC():
    
    def __init__(self,state,realign=True):    
        self.state = state
        self.obstacles = []
        self.realign = realign
    
    def addObstacle(self,vstate):
        maxCommunicationRange = 500   # meters
        packetLossRate = 0 # fraction of messages lost    
        
        # model all sensor errors as Gaussian
        positionErrorSD = 1.   # meters
        speedErrorSD = .5  # meters/second
        #accelErrorSD = .1      # meters/second^2
        # not sending acceleration atm... will take some effort to include        
        
        if vstate.vehID == self.state.vehID: # accidentally sensing yourself
            return
        if distance(self.state, vstate) > maxCommunicationRange:
            return
        if random.uniform(0,1) < packetLossRate:
            return
        
        if self.realign:
            trueObstacle = realignV(self.state, vstate)
        else:
            trueObstacle = vstate.copy()
        trueObstacle.x = random.gauss(trueObstacle.x, positionErrorSD)
        trueObstacle.y = random.gauss(trueObstacle.y, positionErrorSD)
        trueObstacle.speed = random.gauss(trueObstacle.speed, speedErrorSD)
        # for now ignore angle error, since we're dealing with straight roads
        
        
        self.obstacles.append(trueObstacle)

    
    def getObstacles(self):
        return self.obstacles
