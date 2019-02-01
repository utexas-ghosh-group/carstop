#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 8/14/18
get dsrc output -> gps -> position and speed
call read()
output: np.array((x (E-W), y (N-S), speed_x, speed_y))
"""

from prototype.sensors.connectors import RxConnector
import utm
import time
import numpy as np


latlon_convert = 10.**-7

# return the pertinent information from a Basic Safety Message
# input = output of Cohda's bsm-shell application
def hex_to_uint(h):
    return int(h, 16)  
def hex_to_int(h, d):
    i = int(h, 16)
    if i >= 2**(d-1): i -= 2**d
    return i
def parsemessage(message):
    msg = message.split('\n')
    if msg[0][4]=='R':
        Rx = 1
    elif msg[0][4]=='T':
        Rx = 0
    else:
        assert False, "not reading Tx/Rx right, "+msg[0]
    important_part = ' '.join(msg[4:7])
    message_bytes = important_part.split()
    return  (Rx, # 1 received msg, 0 sent msg
             hex_to_uint(message_bytes[0]), # msg_id
             hex_to_uint(''.join(message_bytes[1:5])), # tmp id
             hex_to_uint(''.join(message_bytes[5:7])), # 2 byte current milliseconds
             hex_to_int(''.join(message_bytes[7:11]), 32)*latlon_convert, # 4 byte lat
             hex_to_int(''.join(message_bytes[11:15]), 32)*latlon_convert, # 4 byte long
             hex_to_int(''.join(message_bytes[15:17]), 16), # 2 byte elevation
             hex_to_uint(''.join(message_bytes[17:21])), # 4 byte accuracy
             ''.join((chr(hex_to_uint(h)) for h in message_bytes[38:54]))
            )
    
    
    
gps_delt = .2
gps_R = 2. ** 2 # variance of each position measurement
gps_Qx = .2 ** 2 * gps_delt # random motion
gps_Qv = 2.5 ** 2 * gps_delt # random change in speed
gps_init_Px = gps_R
gps_init_Pv = 30. ** 2

class singleGpsTracker():
    def __init__(self, coord):
        self.pos = np.array(coord)#utm.from_latlon(coord[0], coord[1])[:2]
        self.speed = np.array([0.,0.])
        self.covs_pos = np.array([gps_init_Px]*2)
        self.covs_ps = np.zeros((2,))
        self.covs_speed = np.array([gps_init_Pv]*2)
        #self.last_time = time
		
    def predict(self):
        delt = .2
        #assert delt >= .1999
        #self.last_time = time
        self.pos += self.speed * delt
        self.covs_pos += self.covs_ps * delt * 2 +\
                         self.covs_speed * delt**2 + gps_Qx
        self.covs_ps += self.covs_speed * delt
        self.covs_speed += gps_Qv
	
    def update(self, coord):
        if coord == None:
            pass
        else:
            coord = np.array(coord)#utm.from_latlon(coord[0], coord[1])[:2]
            deviation = coord - self.pos
            precision = 1. / (self.covs_pos + gps_R)
            self.pos += deviation * precision * self.covs_pos
            self.speed += deviation * precision * self.covs_ps
            self.covs_speed -= self.covs_ps**2 * precision
            multiplier = 1 - self.covs_pos * precision
            self.covs_ps *= multiplier
            self.covs_pos *= multiplier
    
    
    
class SelfTrack():
    def __init__(self, port):
        self.conn = RxConnector(port, 'UDP')
        self.me_tracker = None
        self.me_time = 0
        self.me_count = 0
        self.starting = True
        self.last_update_time = None
        
    def __enter__(self):
        return self
    
    def __exit__(self, errtype=None, errval=None, traceback=None):
        self.conn.__exit__()
        
    def get(self):
        
        # read all currently available messages
        while True:
            news = self.conn.recv(1024, .0001)
            if news=='': break
        
            bsm = parsemessage(news)
            lat = bsm[4]
            lon = bsm[5]
            if lat > 90 or lon > 180: continue # gps not active yet
            coord = utm.from_latlon(lat,lon)[:2]
            if bsm[0] == 0:
                # your own message
                # first check if this is a new message or a repeat of previous
                if bsm[3] != self.me_time:
                    self.me_time = bsm[3]
                    self.last_update_time = time.time()
                    if self.starting:
                        self.me_tracker = singleGpsTracker(coord)
                        self.starting = False
                    else:
                        self.me_tracker.predict()
                        self.me_tracker.update(coord)
                        self.me_count += 1
                
        # return current estimates
        me_return = np.append(self.me_tracker.pos, self.me_tracker.speed)
        # predict ahead, because this message might be pretty old
        me_return[:2] += me_return[2:]*(time.time() - self.last_update_time)
        return me_return