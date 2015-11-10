# -*- coding: utf-8 -*-
"""
Collision avoidance model that uses a variant of probabilistic TTC.
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
import math
import numpy
from constants import *

class JoererControl:
    speedMode = 22
    laneChangeMode = 85
    
    def __init__(self, vehID, vParams = None, DELTAT=1.0):
        self.speed = None
        self.accel = None
        self.obstacles = {}
        self.obstaccels = {}
        if vParams == None:
            self.length = 4
            self.width = 2
            self.maxAccel = 1
            self.maxDecel= 1
        else:
            self.length = vParams[0]
            self.width = vParams[1]
            self.maxAccel = vParams[2]
            self.maxDecel = vParams[3]
        self.DELTAT = DELTAT
        
    def updateSpeed(self, speed):
        if self.speed is not None:        
            self.accel = (speed - self.speed) / self.DELTAT
        self.speed = speed
    
    def updateAcceleration(self,obst):
        vid = obst.vehID
        if self.obstacles.has_key(vid):
            self.obstaccels[vid] = (obst.speed -
                                self.obstacles[vid].speed) / self.DELTAT
        else:
            self.obstaccels[vid] = None
        
    def nextStep(self, obstacles):
        newObstacles = {}
        for vstate in obstacles:
            self.updateAcceleration(vstate)
            newObstacles[vstate.vehID] = vstate
        self.obstacles = newObstacles  # gets rid of old vehicles
            
        ## try a number of accelerations, see which have high collision prob.
        resolution = 8
        #probCutoff = .4
        mgfCutoff = 4.0   ## effective TTC
        probCutoff = .001  ## probability at which a TTC of 0 will be still treated as safe
        mgfConstant = pow(probCutoff, -1/mgfCutoff)
        potentialAccels = numpy.arange(-self.maxDecel, self.maxAccel,
                            (self.maxAccel + self.maxDecel)*1.0 / resolution)
        for obstacle in self.obstacles.itervalues():
            for i in range(resolution):
                if potentialAccels[i] > -self.maxDecel:             
                    #if self.PC(potentialAccels[i], obstacle) > probCutoff:
                    if self.MGF(potentialAccels[i], obstacle, mgfConstant) < mgfCutoff:
                        potentialAccels[i] = -self.maxDecel
        choice = max(potentialAccels)
        return self.speed + choice * self.DELTAT
        
    
    def TTC(self, myaccel, obstaccel, obst):
        xSpeed = obst.speed * math.sin(obst.angle)
        ySpeed = obst.speed * math.cos(obst.angle)
        if abs(xSpeed) < EPS:  ## parallel to me
            if abs(obst.x) > (self.width + obst.width)/2:
                return INFINITY
            if obst.y > 0:
                return INFINITY # for now, just ignore problems behind you
                carLength = self.length
            else:
                carLength = -obst.length
            tCross = numpy.roots([(myaccel - obstaccel) / 2,
                                     self.speed - ySpeed,
                                     obst.y - carLength])
            if len(tCross) == 0:
                return INFINITY
            if not numpy.isreal(max(tCross)):
                return INFINITY
            if min(tCross) >= 0:
                return min(tCross)
            return max(tCross)
            
        if abs(ySpeed) < EPS:  ## perpendicular
            tCross = numpy.roots([obstaccel/2, xSpeed, obst.x])
            if not numpy.isreal(max(tCross)):
                return INFINITY
            if min(tCross) >= 0:
                tFrontCross = min(tCross)
            else:
                tFrontCross = max(tCross)
            tBackCross = tFrontCross + obst.length / abs(xSpeed)
            myYFrontCross = self.speed*tFrontCross + myaccel/2*pow(tFrontCross,2)
            myYBackCross = self.speed*tBackCross + myaccel/2*pow(tBackCross,2)
            if myYFrontCross > -obst.y + obst.width/2 + self.length:               
                return INFINITY
            if myYBackCross < -obst.y - obst.width/2:
                return INFINITY
            return tFrontCross  ## poor estimate if obstacle is really slow
        
        return INFINITY  ## figure out the rest later, not in paper anyway
        
    def PC(self, myaccel, obstacle):
        res = 8
        measuredAccel = self.obstaccels[obstacle.vehID]
        if measuredAccel is None or measuredAccel >= obstacle.maxaccel:
            [avals, apdf] = unifDist(-obstacle.maxdecel, obstacle.maxaccel,
                            res)
        else:
            [avals, apdf] = triangDist(-obstacle.maxdecel, obstacle.maxaccel,
                            measuredAccel, res)
        for i in range(len(avals)):
            if not 0 <= self.TTC(myaccel, avals[i], obstacle) < 5:
                apdf[i] = 0
        return sum(apdf)
        
    def MGF(self, myaccel, obstacle, mgfConstant):
        res = 8
        measuredAccel = self.obstaccels[obstacle.vehID]
        [avals, apdf] = unifDist(-obstacle.maxdecel, obstacle.maxaccel, res)
        if measuredAccel is None or measuredAccel >= obstacle.maxaccel:
            [avals, apdf] = unifDist(-obstacle.maxdecel, obstacle.maxaccel,
                            res)
        else:
            [avals, apdf] = triangDist(-obstacle.maxdecel, obstacle.maxaccel,
                            measuredAccel, res)
        mgf = 0
        for i in range(len(avals)):
            ttc = self.TTC(myaccel, avals[i], obstacle)
            if ttc < 0:
                ttc = INFINITY
            mgf += apdf[i] * pow(mgfConstant, -ttc)
        if mgf == 0:
            return INFINITY
        elif mgf < 0:
            print waddufu
        return  -math.log(mgf) / math.log(mgfConstant)
        
def unifDist(amin, amax, res):
    x = numpy.arange(amin, amax, (amax - amin)*1.0/res)
    fx = [1.0 / x.size] * x.size
    return [list(x), fx]
    
def triangDist(amin, amax, acenter, res):
    x = numpy.arange(amin, amax, (amax - amin)*1.0/res)
    peakhigh = 2.0 / (amax - amin)
    if max(x) > acenter:
        peakloc = [i > acenter for i in x].index(True)
    else:
        peakloc = x.size
    if amin == acenter:
        lefthalf = x.copy() * peakhigh
    else:
        lefthalf = (x.copy() - amin) * peakhigh / (acenter - amin)
    if amax == acenter:
        righthalf = x.copy() * peakhigh
    else:
        righthalf = (x.copy() - amax) * peakhigh / (acenter - amax)
    fx = numpy.append(lefthalf[0:peakloc], righthalf[peakloc:x.size])
    return [list(x), list(fx)]