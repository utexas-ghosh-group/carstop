#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gives a vehicle approximate knowledge about others, if they are close enough.
Note that it realigns them as if this vehicle has
position (0,0) and angle 0.
Last modified 6/20/15
"""
#import sys, os
#sys.path.append(os.path.dirname(__file__)[:-len("/Sensors")])
#from constants import *
from usefulMethods import realignV, distance
import random

class BasicSensor():
    
    def __init__(self,state):    
        self.state = state
        self.obstacles = []
    
    def addObstacle(self,vstate):
        radius = 20   # meters
        positionSD = .25
        
        if vstate.vehID == self.state.vehID:
            return
        if distance(self.state, vstate) > radius:
            return
        trueObstacle = realignV(self.state, vstate)
        trueObstacle.x = random.gauss(trueObstacle.x, positionSD)
        trueObstacle.y = random.gauss(trueObstacle.y, positionSD)
        self.obstacles.append(trueObstacle)
    
    def getObstacles(self):
        return self.obstacles
