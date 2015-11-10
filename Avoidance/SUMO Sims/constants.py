# -*- coding: utf-8 -*-
"""
This is a storage file for useful constants and functions.
Last modified 6/10/15

Based off of:
@file    constants.py
@author  Michael Behrisch
@author  Daniel Krajzewicz
@date    2008-07-21
@version $Id: constants.py 16379 2014-05-14 09:28:38Z behrisch $
"""
import os, sys

''' put your SUMO tools directory here '''
toolsPathName = "../../../cars/sumo/tools"
sys.path.append(os.path.realpath(toolsPathName))


PORT = 8800
INFINITY = 1e400
EPS = .0001
MAXSTEPS = 10000

try:
    from sumolib import checkBinary
except ImportError:
    def checkBinary(name):
        return name
NETCONVERT = checkBinary("netconvert")
SUMO = checkBinary("sumo")
SUMOGUI = checkBinary("sumo-gui")


class VState:
    # meant to pass all necessary information about vehicles between
    # scripts.  Can take either listed input, or one-by-one input.
    def __init__(self, firstIn, x=None, y=None, angle=None, speed=None,
                 length=None, width=None, maxaccel=None, maxdecel=None):        
        self.vehID = firstIn
        firstInLen = 1        
        if isinstance(firstIn, list):
            self.vehID = firstIn[0]            
            firstInLen = len(firstIn)
        
        if x == None and firstInLen > 1:
            self.x = firstIn[1]
        else:
            self.x = x
            
        if y == None and firstInLen > 2:
            self.y = firstIn[2]
        else:
            self.y = y
            
        if angle == None and firstInLen > 3:
            self.angle = firstIn[3]
        else:
            self.angle = angle
            
        if speed == None:
            if firstInLen > 4:
                self.speed = firstIn[4]
            else:
                self.speed = 0
        else:
            self.speed = speed
            
        if length == None:
            if firstInLen > 5:
                self.length = firstIn[5]
            else:
                self.length = 4
        else:
            self.length = length
            
        if width == None:
            if firstInLen > 6:
                self.width = firstIn[6]
            else:
                self.width = 2
        else:
            self.width = width
            
        if maxaccel == None:
            if firstInLen > 7:
                self.maxaccel = firstIn[7]
            else:
                self.maxaccel = 1.0
        else:
            self.maxaccel = maxaccel
            
        if maxdecel == None:
            if firstInLen > 8:
                self.maxdecel = firstIn[8]
            else:
                self.maxdecel = 1.0
        else:
            self.maxdecel = maxdecel
    
    def copy(self):
        return VState(self.vehID, self.x, self.y, self.angle,
                      self.speed, self.length, self.width,
                      self.maxaccel, self.maxdecel)
    
    def __repr__(self):
        return "(%s,%s) angle:%s" % (self.x, self.y, self.angle)
