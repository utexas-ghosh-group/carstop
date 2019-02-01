# -*- coding: utf-8 -*-
"""
last mod 8/13/18

returns list of messages with five fields
1: 1 if received message, 0 if your own
2: temporary unique vehicle id if received message, 0 if your own
3: x (E-W UTM)
4: y (N-S UTM)
5: 16-character string, extra message space
"""

from connectors import RxConnector
import utm
from multiprocessing import Process, Queue

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
    raw_message = (Rx, # 1 received msg, 0 sent msg
            hex_to_uint(message_bytes[0]), # msg_id
            hex_to_uint(''.join(message_bytes[1:5])), # tmp id
            hex_to_uint(''.join(message_bytes[5:7])), # 2 byte current seconds
            hex_to_int(''.join(message_bytes[7:11]), 32), # 4 byte lat
            hex_to_int(''.join(message_bytes[11:15]), 32), # 4 byte long
            hex_to_int(''.join(message_bytes[15:17]), 16), # 2 byte elevation
            hex_to_uint(''.join(message_bytes[17:21])), # 4 byte accuracy
            ''.join((chr(hex_to_uint(h)) for h in message_bytes[38:54]))
            ) # 16 extra bytes added for other messages
    return raw_message
def hex_to_uint(h):
    return int(h, 16)  
def hex_to_int(h, d):
    i = int(h, 16)
    if i >= 2**(d-1): i -= 2**d
    return i



class DSRC(Process):
    def __init__(self, port):
        Process.__init__(self)
        self.conn = RxConnector(port, 'UDP')
        self.queue = Queue()
        
    def __enter__(self):
        return self.queue
        
    def run(self):
        my_last_time = 0
        my_last_msg = None
        times = []
        msgs = []
        ids = []
        updated = []
        
        while True:
            news = self.conn.recv(1024, .2)
            assert news!='', "DSRC connection stopped"
            rcv, msg_id, tmp_id, snd_time, lat, lon, elevation, accuracy,\
                        extra = parsemessage(news)
            
            lat *= 10.**-7
            lon *= 10.**-7
            if lat < 90 and lon < 180:
                x, y = utm.from_latlon(lat,lon)[:2]
                
                if rcv==0:
                    if snd_time > my_last_time or snd_time < my_last_time - 50:
                        my_last_msg = (x, y, extra)
                        my_last_time = snd_time
                    
                        # send current messages
                        output = [(1, vid, msg[0], msg[1], msg[2])
                                for vid, msg, update in zip(ids, msgs, updated)
                                if update]
                        output.append((0, 0, my_last_msg[0], my_last_msg[1],
                                       my_last_msg[2]))
                        self.queue.add(output)
                
                else:
                    found = False
                    for idx, current_id in enumerate(ids):
                        if current_id == tmp_id:
                            if snd_time > times[idx] or snd_time < times[idx] - 50:
                                msgs[idx] = (x, y, extra)
                                times[idx] = snd_time
                                updated[idx] = True
                            found=True
                            break
                    if not found:
                        times.append(snd_time)
                        ids.append(msg_id)
                        msgs.append((x, y, extra))
                        updated[idx].append(True)
                    
    def terminate(self):
        self.conn.__exit__()
        self.queue.close()
        super(DSRC, self).terminate()
                
    def __exit__(self, errtype=None, errval=None, traceback=None):
        self.terminate()