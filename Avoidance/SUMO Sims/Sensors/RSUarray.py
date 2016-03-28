# -*- coding: utf-8 -*-
"""
3/28/16
"""
import sys, os
from numpy import arctan2, pi
from pandas import Series
import random
from usefulMethods import realignV, distance
sys.path.append(os.path.realpath(__file__)[:-len("/Sensors/RSUarray.py")])

class RSUarray():
    def __init__(self, state, realign, units, noiseLevel=0., packetLossRate=0.,
                 maxCommunicationRange=600.):
        self.state = state
        self.obstacles = []
        self.realign = realign
        self.units = units
        self.noiseLevel = noiseLevel
        self.maxCommunicationRange = maxCommunicationRange
        self.packetLossRate = packetLossRate
        
    def _withinRSUrange(self, unit, vstate):
        difference = (vstate.x - unit[0], vstate.y - unit[1])
        distance = (difference[0]**2+difference[1]**2)**.5
        angle = arctan2(difference[1],difference[0])
        return distance < 7.5 and abs(angle - unit[2]) < pi/4
            
        
    def addObstacle(self, vstate):
                # model all sensor errors as Gaussian
        positionErrorSD = 2. * self.noiseLevel / 2.  # meters
        speedErrorSD = 2.*.447 * self.noiseLevel / 2. # meters/second
        #angleErrorSD = .1 * self.noiseLevel       # radians
        #accelErrorSD = .2 * self.noiseLevel      # meters/second^2
        # not sending acceleration atm...       
        
        unitThatWorks = None
        unitsThatWork = (unit for unit in self.units if
                                self._withinRSUrange(unit, vstate))
        try:
            while unitThatWorks is None:
                unit = unitsThatWork.next()
                #unitState = {'x':unit[0], 'y':unit[1], 'angle':unit[2]}
                unitState = Series(unit,index=['x','y','angle'])                
                if distance(self.state, unitState) <= self.maxCommunicationRange:
                    unitThatWorks = unit
        except StopIteration:
            return   # no units detected this vehicle, and can communicate

        if random.uniform(0,1) < self.packetLossRate:
            return
        
        if self.realign:
            trueObstacle = realignV(self.state, vstate)
        else:
            trueObstacle = vstate.copy()
        trueObstacle.x = random.gauss(trueObstacle.x, positionErrorSD)
        trueObstacle.y = random.gauss(trueObstacle.y, positionErrorSD)
        trueObstacle.speed = random.gauss(trueObstacle.speed, speedErrorSD)
        #trueObstacle.Acceleration is same
        # for now ignore angle error, since we're dealing with straight roads
        
        self.obstacles.append(trueObstacle)
        
               
    def getObstacles(self):
        return self.obstacles
