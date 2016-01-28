#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Like DSRC, but with range primarily in the front (of the ego)
Note that it realigns them as if this vehicle has
position (0,0) and angle 0.
Last modified 1/27/16
"""
import sys, os
sys.path.append(os.path.realpath(__file__)[:-len("/Sensors/DSRC.py")])
from constants import *
from usefulMethods import realignV
import random
from math import atan2, pi

class FrontRadar():
    
    def __init__(self,state,realign, noiseLevel=1., maxCommunicationRange=500.,
                 packetLossRate=0., scope=pi):    
        self.state = state
        self.obstacles = []
        self.realign = realign
    
#    def inRange(self,vstate): # a rectangular model
#        forwardRange = 30.
#        backwardRange = 5.
#        sideRange = 5.
#        left  = vstate.x > -2/2. - sideRange
#        right = vstate.x < 2/2. + sideRange
#        above = vstate.y < 5. + backwardRange
#        below = vstate.y > -forwardRange
#        return left and right and above and below
    
    def inRange(self, vstate): # conical model
        angleOfObject = atan2(vstate.y, vstate.x) + pi/2
        if angleOfObject > pi:
            angleOfObject = angleOfObject - 2*pi
        return angleOfObject > -self.scope and angleOfObject < self.scope
    
    def addObstacle(self,vstate):
        packetLossRate = 0 # fraction of messages lost    
        
        # model all sensor errors as Gaussian
        positionErrorSD = 4. * self.noiseLevel / 2.  # meters
        speedErrorSD = 5.*.447 * self.noiseLevel / 2. # meters/second
        
        if vstate.vehID == self.state.vehID: # accidentally sensing yourself
            return
        if not self.inRange(realignV(self.state,vstate)):
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
        # for now ignoring angle error
        
        
        self.obstacles.append(trueObstacle)
    
    def getObstacles(self):
        return self.obstacles