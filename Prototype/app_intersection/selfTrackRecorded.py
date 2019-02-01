#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 1/10/19
reads file obtained from selfTrack.py in the Carstop data analysis code
initially came from GPS+IMU data
returns: x,y position, heading (angle), speed, acceleration, angular velocity
"""
import pandas as pd
from prototype.app_intersection.options import recorded_track_file

class SelfTrack():
    def __init__(self):
        self.data = pd.read_csv(recorded_track_file, header=0)
        self.idx = 5027 # start of the example
    
    def __enter__(self):
        return self
    
    def __exit__(self, errtype=None, errval=None, traceback=None):
        pass
        
    def get(self):
        self.idx += 1
        return self.data.iloc[self.idx].copy()