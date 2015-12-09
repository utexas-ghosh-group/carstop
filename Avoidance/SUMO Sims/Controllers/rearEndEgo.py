# -*- coding: utf-8 -*-
"""
remains at the speed at which it was created
11/9/15
"""
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
#from constants import * # sys.path is modified here

class RearEndEgo:
    
    def __init__(self,speed, accel=0):
        self.speed = speed
        self.accel = accel
        
    def update(self, DELTAT):
        self.speed = self.speed + self.accel*DELTAT 
    
    def nextStep(self):
        return self.speed