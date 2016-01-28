#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Adds Gaussian noise, and removes vehicles out of a circular range.
Can also remove random instances with packet loss.
Last modified 1/27/16
"""
import sys, os
sys.path.append(os.path.realpath(__file__)[:-len("/Sensors/DSRC.py")])
from constants import *
from usefulMethods import realignV, distance
import random

class DSRC():
    
    def __init__(self,state,realign,noiseLevel=1.,
                 maxCommunicationRange=500.,
                 packetLossRate=0.):    
        self.state = state
        self.obstacles = []
        self.realign = realign
        self.noiseLevel = noiseLevel # ratio of max sensor noises
        self.maxCommunicationRange = maxCommunicationRange # meters
        self.packetLossRate = packetLossRate # fraction of messages lost, 0-1 
    
    def addObstacle(self,vstate):
        
        # model all sensor errors as Gaussian
        positionErrorSD = 4. * self.noiseLevel / 2.  # meters
        speedErrorSD = 5.*.447 * self.noiseLevel / 2. # meters/second
        #angleErrorSD = .1 * self.noiseLevel       # radians
        #accelErrorSD = .2 * self.noiseLevel      # meters/second^2
        # not sending acceleration atm...       
        
#        if vstate.vehID == self.state.vehID: # accidentally sensing yourself
#            return
        if distance(self.state, vstate) > self.maxCommunicationRange:
            return
        if random.uniform(0,1) < self.packetLossRate:
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
