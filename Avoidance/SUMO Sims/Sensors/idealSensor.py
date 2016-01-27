#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gives a vehicle complete knowledge about the others.
Note that it realigns them as if this vehicle has
position (0,0) and angle 0.
Last modified 1/27/16
"""
#import sys, os
#sys.path.append(os.path.dirname(__file__)[:-len("/Sensors")])
#from constants import *
from usefulMethods import realignV

class IdealSensor():
    
    def __init__(self, state, realign):    
        self.state = state
        self.obstacles = []
        self.realign = realign
    
    def addObstacle(self,vstate):
        if self.realign:
            self.obstacles.append(realignV(self.state, vstate))
        else:
            self.obstacles.append(vstate)
    
    def getObstacles(self):
        return self.obstacles