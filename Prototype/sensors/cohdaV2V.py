# -*- coding: utf-8 -*-
"""
last mod 8/14/18

returns list of messages with five fields
1: 1 if received message, 0 if your own
2: temporary unique vehicle id if received message, 0 if your own
3: x (E-W UTM)
4: y (N-S UTM)
5: 16-character string, extra message space


This sensor is not processed in a separate thread for two reasons:
1. It's a low quantity of information compared to lidar/camera
2. It updates less frequently (.2 seconds) than the update rate of the prototype
instead it uses select() to check if new information is there at every update
hypothetically, it might return the exact same information twice
"""

from connectors import RxConnector
import utm
from trackers import singleGpsTracker as GpsTracker


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
    
    
class V2V():
    def __init__(self, port):
        self.conn = RxConnector(port, 'UDP')
        self.me_tracker = None
        self.me_time = 0
        self.me_count = 0
        self.starting = True
        self.trackers = []
        self.counts = []
        self.times = []
        self.ids = []
        self.counts = []
        
    def __enter__(self):
        return self
    
    def __exit__(self, errtype=None, errval=None, traceback=None):
        self.conn.__exit__()
        
    def read(self):        
        n_others = len(self.ids)
        keep_other = [count < self.me_count + 50 for count in self.counts]
        self.trackers = [self.trackers[k] for k in range(n_others) if keep_other[k]]
        self.ids = [self.ids[k] for k in range(n_others) if keep_other[k]]
        self.times = [self.times[k] for k in range(n_others) if keep_other[k]]
        self.counts = [self.counts[k] for k in range(n_others) if keep_other[k]]        
        
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
                    if self.starting:
                        self.me_tracker = GpsTracker(coord)
                        self.starting = False
                    else:
                        self.me_tracker.predict()
                        self.me_tracker.update(coord)
                        self.me_count += 1
            else:
                otherID = bsm[2]
                othertime = bsm[3]
                included = False
                for k in range(len(self.trackers)):
                    if self.ids[k] == otherID:
                        included = True
                        time_diff = othertime - self.times[k]
                        if time_diff < -59000: time_diff += 60000 # correct for minute
                        if time_diff > 0:
                            self.times[k] = othertime
                            tracker = self.trackers[k]
                            # might need to predict more than once
                            # because a message was skipped
                            for tk in range(0,time_diff,200):
                                tracker.predict()
                            tracker.update(coord)
                            self.counts[k] = self.me_count
                if not included: # new vehicle detected
                    self.ids.append(otherID)
                    self.trackers.append(GpsTracker(coord))
                    self.times.append(othertime)
                    self.counts.append(self.me_count)
                    assert len(self.counts) == len(self.trackers)
                    assert len(self.counts) == len(self.times)
             
                
        # return current estimates
        me_return = list(self.me_tracker.pos) + list(self.me_tracker.speed)
        other_return = [(
                list(self.trackers[k].pos) + list(self.trackers[k].speed),
                self.ids[k]) for k in range(len(self.trackers))]
        return me_return, other_return