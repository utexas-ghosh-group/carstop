# -*- coding: utf-8 -*-
"""
last modified 5/10/18
"""

from prototype.sensors.connectors import RxConnector
import utm
from prototype.trackers import singleGpsTracker as GpsTracker
from math import hypot

# port that Cohda is set to output to
from prototype.app_highwaymerge.options import port

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
    return  [Rx, # 1 received msg, 0 sent msg
             hex_to_uint(message_bytes[0]), # msg_id
             hex_to_uint(''.join(message_bytes[1:5])), # tmp id
             hex_to_uint(''.join(message_bytes[5:7])), # 2 byte current milliseconds
             hex_to_int(''.join(message_bytes[7:11]), 32)*10.**-7, # 4 byte lat
             hex_to_int(''.join(message_bytes[11:15]), 32)*10.**-7, # 4 byte long
             hex_to_int(''.join(message_bytes[15:17]), 16), # 2 byte elevation
             hex_to_uint(''.join(message_bytes[17:21])), # 4 byte accuracy
             #message_bytes[38:54])),
             ] 


			 
conn = RxConnector(port, 'UDP')

me = {'tracker':None, 'time':0, 'count':0, 'starting':True}
other = {'trackers' : [], 'times' : [], 'ids' : [], 'counts' : []}

def startSensor():
    pass

def stopSensor():
    conn.__exit__()
    
## old tracker code
#lasttime = [0]
#def readSensorOld():
#    othermsgs = []
#    while True:
#        news = conn.recv(1024, .2)
#        assert news!='', "DSRC connection stopped"
#        bsm = parsemessage(news)
#        lat = bsm[4]*10.**-7
#        lon = bsm[5]*10.**-7
#        if lat > 90 or lon > 180: continue
#        coord = utm.from_latlon(lat,lon)[:2]
#        if bsm[0] == 0:
#            if bsm[3] != lasttime[0]:
#                lasttime[0] = bsm[3]
#                return coord, [othercoord for otherID, othertime, othercoord in othermsgs]
#        else:
#            otherID = bsm[2]
#            included = False
#            for k, othermsg in enumerate(othermsgs):
#                if othermsgs[0] == otherID:
#                    included = True
#                    if bsm[3] > othermsgs[1] or bsm[3] < othermsgs[1] - 59000:
#                        othermsgs[k] = (otherID, bsm[3], coord)
#            if not included:
#                othermsgs.append((otherID, bsm[3], coord))



# current setup:
# read until you get a new message from self
# latest message from each detected vehicle will be included            
# vehicles that don't send messages for more than 10 seconds are removed
def readSensor():
    # delete tracked vehicles that have not been detected in 10 seconds
    n_others = len(other['ids'])
    keep_other = [count < me['count'] + 50 for count in other['counts']]
    other['trackers'] = [other['trackers'][k] for k in range(n_others) if keep_other[k]]
    other['ids'] = [other['ids'][k] for k in range(n_others) if keep_other[k]]
    other['times'] = [other['times'][k] for k in range(n_others) if keep_other[k]]
    other['counts'] = [other['counts'][k] for k in range(n_others) if keep_other[k]]        
    
    while True:
        news = conn.recv(1024, .2)
        assert news!='', "DSRC connection stopped"
        bsm = parsemessage(news)
        lat = bsm[4]
        lon = bsm[5]
        if lat > 90 or lon > 180: continue # gps not active yet
        coord = utm.from_latlon(lat,lon)[:2]
        if bsm[0] == 0:
            # your own message
            # first check if this is a new message or a repeat of previous
            if bsm[3] != me['time']:
                me['time'] = bsm[3]
                if me['starting']:
                    me['tracker'] = GpsTracker(coord)
                    me['starting'] = False
                else:
                    me['tracker'].predict()
                    me['tracker'].update(coord)
                    me['count'] += 1
                # return current estimates
                me_return = (me['tracker'].pos.copy(),
                             hypot(*me['tracker'].speed))
                other_return = [(other['trackers'][k].pos.copy(),
                                 hypot(*other['trackers'][k].speed),
                                 other['ids'][k]) for k in range(len(other['trackers']))]
                return me_return, other_return
        else:
            otherID = bsm[2]
            othertime = bsm[3]
            included = False
            for k in range(len(other['trackers'])):
                if other['ids'][k] == otherID:
                    included = True
                    time_diff = othertime - other['times'][k]
                    if time_diff < -59000: time_diff += 60000 # correct for minute
                    if time_diff > 0:
                        other['times'][k] = othertime
                        tracker = other['trackers'][k]
                        # might need to predict more than once
                        # because a message was skipped
                        for tk in range(0,time_diff,200):
                            tracker.predict()
                        tracker.update(coord)
                        other['counts'][k] = me['count']
            if not included: # new vehicle detected
                other['ids'].append(otherID)
                other['trackers'].append(GpsTracker(coord))
                other['times'].append(othertime)
                other['counts'].append(me['count'])
                assert len(other['counts']) == len(other['trackers'])
                assert len(other['counts']) == len(other['times'])
