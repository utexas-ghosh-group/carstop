#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 8/24/18
"""

import numpy as np
import socket, select, struct
from multiprocessing import Process, Queue#, Manager
from Queue import Empty


#cutoff_min_height = 0.#.5
#cutoff_max_height = 2.3#2.1
#cutoff_min_distance = 2.#2.3
#cutoff_max_distance = 100.#50.
min_dist_to_process = 1.
max_dist_to_process = 200.

preamble = ''.join((chr(int(byte, 16)) for byte in ['75','bd','7e','97']))
preamble += struct.pack(">L", 6632)
firingidxlist = [firing*132 for firing in range(50)]
fullpackedstructure = ">H8L"
angle_conversion = np.pi / 5200.
distance_conversion = 10.**-5
vertical_angles = np.array((-.318505,-.2692,-.218009,-.165195,-.111003,
                            -.0557982,0.,.0557982))
vertical_components = np.sin(vertical_angles) * distance_conversion
horizontal_components = np.cos(vertical_angles) * distance_conversion
vertical_transform = np.tan(vertical_angles)


def _readFromLidar(msg, lasers):
    assert msg[:8] == preamble,\
        "LIDAR status packet "+','.join((str(ord(char)) for char in msg[:8]))
    
    data = []
    for i in firingidxlist:
        firinglist = struct.unpack(fullpackedstructure,
                                msg[i+20:i+22]+msg[i+24:i+56])
        angle = firinglist[0]*angle_conversion
        cos = np.cos(angle)
        if cos < 0: # not towards front of vehicle
            continue
        sin = np.sin(angle)
        for j in lasers: # don't bother with first laser
            dist = firinglist[1+j] * horizontal_components[j]
            if dist < min_dist_to_process or dist > max_dist_to_process:
                continue
            x = dist * cos
            y = dist * sin
            z = firinglist[1+j] * vertical_components[j]
            data.append((x,y,z)) 
    return data


class LIDAR(Process):
    def __init__(self, IP='10.1.11.124', port=4141, lasers = [4,5,6]):
        Process.__init__(self)
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.IP = IP
        self.port = port
        self.queue = Queue()
        self.gap_time = True
        
    def __enter__(self):
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        print("connecting to LIDAR...")
        self.socket.connect((self.IP, self.port))
        print "LIDAR connected"
        print("waiting for first LIDAR packet...")
        ready_to_read, ready_to_write, in_error = select.select(
              (self.socket,), [], [], 30.)
        assert len(ready_to_read) > 0, "LIDAR not running within 30 seconds"
        print("LIDAR sending packets")
        
        # do one read so that all future reads are complete rotations
        msg = []
        read = '1'
        while len(read) > 0:
            while len(msg) < 6632:
                msg += self.socket.recv(4096)
            read = _readFromLidar(msg[:6632])
            msg = msg[6632:]
            self.leftovermsg = msg
            
        Process.start(self)
        return self
    
    # timeout should probably be .05?
    def get(self, timeout=.05):
        try:
            points = self.queue.get(timeout=timeout)
        except Empty: # no rotation available, just wait
            assert not self.gap_time
            self.gap_time = True
            return self.points.copy()
        try:
            points = self.queue.get(block=False) # grab extra waiting rotations
        except Empty: pass
        self.gap_time = False
        self.points = points
        return points.copy()
    
    # this is the part that runs in a separate thread - or at least it should
    def run(self):
        rotation = []
        msg = []
        on_back_end = True
        while True:
            readable,a,b = select.select(self.select_sock, [], [], .1)
            assert len(readable) > 0, "LIDAR rx timeout"
            while len(msg) < 6632:
                msg += self.socket.recv(4096)
            read = _readFromLidar(msg[:6632])
            if len(read) == 0:
                on_back_end = True
            elif not on_back_end:
                rotation += read
            else: # rotation complete, for our purposes
                on_back_end = False
                self.queue.put(np.array(rotation))
                rotation = read
            msg = msg[6632:]
    
    def __exit__(self, errtype=None, errval=None, traceback=None):
        if not (errtype is None or errtype is KeyboardInterrupt or
                                   errtype is SystemExit):
            print(errtype)
            print(errval)
        self.terminate()
        
    def terminate(self):
        self.sock.close()
        self.queue.close()
        super(LIDAR, self).terminate()