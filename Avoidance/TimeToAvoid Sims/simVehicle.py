# -*- coding: utf-8 -*-
"""
Created on Thu Sep 24 17:57:47 2015

@author: motro
"""
import numpy as np

VEHlength = 5.0
VEHwidth = 2.0

class Vehicle:
    # meant to pass all necessary information about vehicles between
    # scripts.  Can take either listed input, or one-by-one input.
    def __init__(self, x=0, y=0, speed=0, head=0, accel=0, wheel=0):        
        self.x=x # East-West position in meters (front-center of car)
        self.y=y # North-South position in meters (front-center of car)
        self.speed=speed # magnitude of velocity in m/s
        self.head=head # heading in radians (0 is East)
        self.accel=accel # change in speed in m/s^2
        self.wheel=wheel # wheel angle in rad (or angular velocity in rad/s)
        
    def center(self):
        return (self.x - VEHlength/2*np.cos(self.head), self.y - VEHlength/2*np.sin(self.head))

    def moveSelf(self, t): # t can only be a scalar for now
        newState = self.moveCA(t)
        self.x = newState[0]
        self.y = newState[1]
        self.head = newState[2]
        self.speed = newState[3]
        #self.accel = np.repeat(self.accel, len(t))
        #self.wheel = np.repeat(self.wheel, len(t))
        
    def copy(self):
        return Vehicle(self.x, self.y, self.speed, self.head,
                      self.accel, self.wheel)
                      
                      
    def moveCA(self, t): # constant acceleration + angular velocity
        # t can be scalar or numpy array
        # wheel actualy angular velocity
        a = self.accel
        w = self.wheel
        v0 = self.speed
        a0 = self.head
        osin = np.sin(a0)
        ocos = np.cos(a0)
        speed = v0 + a*t
        head = a0 + w*t
        if w > .001:
            x = self.x + speed*np.sin(head)/w - v0/w*osin + a/w/w*(np.cos(head) - ocos)
            y = self.y - speed*np.cos(head)/w + v0/w*ocos + a/w/w*(np.sin(head) - osin)
        else:  # second degree Taylor, gets rid of w's in determinant
            x = self.x + t*(v0+a*t/2)*ocos - w*t*t/2*speed*osin
            y = self.y + t*(v0+a*t/2)*osin + w*t*t/2*speed*ocos
        return (x, y, head, speed)