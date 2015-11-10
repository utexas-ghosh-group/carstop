# -*- coding: utf-8 -*-
"""
Created on Wed Jun 17 17:03:30 2015

@author: motro
"""
from constants import *
import traci.constants as tc
import random

def randomize(varID, varValue):  
    
    #if state is not None:
    #    random.setstate(state)
    
    if varID == tc.VAR_ACCEL:
        return max(random.gauss(varValue, 1.0), 0.5)
    else:
        return varValue