#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Modeled after real radar unit spec sheet discovered by Taewan
Last modified 2/9/16
"""
import sys, os
sys.path.append(os.path.realpath(__file__)[:-len("/Sensors/delphiESR.py")])
from usefulMethods import realignV, distance
import random
from math import atan2, pi

class DelphiESR():
    
    def __init__(self,state,realign, allowComm=False):    
        self.state = state
        self.obstacles = []
        self.realign = realign
        self.commRange1 = 174.
        self.commRange2 = 60.
        self.scope1 = pi / 18.
        self.scope2 = pi / 4.
        self.allowComm = allowComm
    
    def inScope(self, vstate, scope): # conical model
        realignedV = realignV(self.state, vstate)
        angleOfObject = atan2(realignedV.y, realignedV.x) + pi/2
        if angleOfObject > pi:
            angleOfObject = angleOfObject - 2*pi
        return angleOfObject >= -scope and angleOfObject <= scope
    
    def addObstacle(self,vstate):
        
        # model all sensor errors as Gaussian
        positionErrorSD = 4. * .25 / 2.  # meters
        speedErrorSD = 5.*.447 * .25 / 2. # meters/second
        
        detected = (distance(self.state, vstate) <= self.commRange1 and
                    self.inScope(vstate,self.scope1) ) or (
                    distance(self.state, vstate) <= self.commRange2 and
                    self.inScope(vstate,self.scope2) )
        
        if not detected:
            if self.allowComm:   # using DSRC instead
                positionErrorSD = 4. * 1.1 / 2.
                speedErrorSD = 5.*.447 * 1.1 / 2.
            else:
                return
        
        if self.realign:
            trueObstacle = realignV(self.state, vstate)
        else:
            trueObstacle = vstate.copy()
        trueObstacle.x = random.gauss(trueObstacle.x, positionErrorSD)
        trueObstacle.y = random.gauss(trueObstacle.y, positionErrorSD)
        trueObstacle.speed = random.gauss(trueObstacle.speed, speedErrorSD)
        
        
        self.obstacles.append(trueObstacle)
    
    def getObstacles(self):
        return self.obstacles