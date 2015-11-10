# -*- coding: utf-8 -*-
"""
Simple example of collision-avoidance model.  Stores all permanent
characteristics upon construction, and takes its speed and the position of other objects
(relative to this one) every step.  Returns the speed for the next turn.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import * # sys.path is modified here
import math

class XiangControl:
    def __init__(self, vehID, vParams = None):
        self.ID = vehID
        self.speed = None
        self.obstacles = {}
        if vParams == None:
            self.length = 4
            self.width = 2
            self.maxAccel = 1
        else:
            self.length = vParams[0]
            self.width = vParams[1]
            self.maxAccel = vParams[2]
        
    def updateSpeed(self, speed):
        self.speed = speed
    
    def nextStep(self, obstacles):
        if straight:
        
        
        speed = self.speed + self.maxAccel
        for vstate in obstacles:
            self.obstacles[vstate.vehID] = vstate
            if vstate.vehID != self.ID:
                if 0 <= self.TTC(speed, vstate) < 3:
                    speed = self.speed
                if 0 <= self.TTC(speed, vstate) < 3:
                    speed = 0
        return speed
    
    def TTC(self, speed, obst):      
        if (obst.angle + EPS) % math.pi < 2*EPS:
            if obst.speed == speed:
                return INFINITY
            if obst.y > 0: 
                return (obst.y - self.length)*1.0 / (obst.speed - speed)
            return (obst.y + obst.length)*1.0 / (obst.speed - speed)
        tCross = -obst.x / (obst.speed * math.sin(obst.angle))
        yMe = -tCross * speed
        yObstacle = obst.y - obst.speed * math.cos(obst.angle) * tCross
        minDistance = pow(pow(obst.length,2)+pow(obst.width,2)/4.0,.5) + self.length
        if abs(yMe - yObstacle) + EPS < minDistance:
            return tCross
        return INFINITY
    
    def straight(obstacle):
        # returns True if obstacle is facing same way as self
        # approximation due to rounding from math.radian, etc.
        return (obstacle.angle + EPS) % math.pi < 2*EPS
    
    def MinSafeDistance(self, speed, other):
        dToStopSelf = pow(speed,2) / self.maxdecel / 2.0
        dToStopOther = pow(other,2) / other.maxdecel / 2.0
        return other.length + dToStopSelf - dToStopOther