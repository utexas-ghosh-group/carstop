#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 11/9/18

Unlike the previous quanergyLidar code, this code removes all data.

computer's IP must be at 10.1.11.XXX

function = get() or get(timeout)
timeout should probably not be set unless you have a good reason

output = tuple of variable length, of:
             (x,y,z) tuple of each laser point

If run on its own, this file will plot sensor output in real time, as a test
press ctrl-c to stop it
"""

from math import cos,sin,tan,pi
import socket, select, struct
from multiprocessing import Process, Queue#, Manager
from Queue import Empty
from time import time, sleep

from prototype.sensors.proprietary.quanergyParams import preamble, firingidxlist,\
     fullpackedstructure, angle_conversion, distance_conversion, vertical_angles

## these settings select a certain segment of data to be output
## they can also be achieved by changing the settings of the Quanergy unit
# output everything
#lasers = [0,1,2,3,4,5,6,7] # which lasers (0 through 7) to use
#min_dist_to_process = 1. # this is the lowest distance that the lidar returns
#max_dist_to_process = 200.
#min_angle_to_process = 0.
#max_angle_to_process = 2*pi

lasers = [5,6] 
min_dist_to_process = 1. # this is the lowest distance that the lidar returns
max_dist_to_process = 10.
min_angle_to_process = 0.
max_angle_to_process = 2*pi

vertical_components = [sin(angle) * distance_conversion for angle in vertical_angles]
horizontal_components = [cos(angle) * distance_conversion for angle in vertical_angles]
vertical_transform = [tan(angle) for angle in vertical_angles]

wrap = max_angle_to_process < min_angle_to_process # wraps around 0

def _readFromLidar(msg):
    assert msg[:8] == preamble,\
        "LIDAR status packet "+','.join((str(ord(char)) for char in msg[:8]))
    
    data = []
    for i in firingidxlist:
        firinglist = struct.unpack(fullpackedstructure,
                                msg[i+20:i+22]+msg[i+24:i+56])
        angle = firinglist[0]*angle_conversion
        if wrap and (angle < min_angle_to_process and angle > max_angle_to_process):
            continue
        elif not wrap and (angle < min_angle_to_process or angle > max_angle_to_process):
            continue
        c = cos(angle)
        s = sin(angle)
        for j in lasers: # don't bother with first laser
            dist = firinglist[1+j] * horizontal_components[j]
            if dist < min_dist_to_process or dist > max_dist_to_process:
                continue
            x = dist * c
            y = dist * s
            z = firinglist[1+j] * vertical_components[j]
            data.append((x,y,z)) 
    return data, angle



class LIDAR(Process):
    def __init__(self, IP='10.1.11.129', port=4141, max_gap_time = .3):
        Process.__init__(self)
        self.socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.IP = IP
        self.port = port
        self.queue = Queue()
        self.points = ()
        self.time = 0.
        self.max_gap_time = max_gap_time
        
    def __enter__(self):
        print("connecting to LIDAR...")
        self.socket.connect((self.IP, self.port))
        print "LIDAR connected"
        print("waiting for first LIDAR packet...")
        ready_to_read, ready_to_write, in_error = select.select(
              (self.socket,), [], [], 30.)
        assert len(ready_to_read) > 0, "LIDAR not running within 30 seconds"
        print("LIDAR sending packets")
        
        # do one read so that all future reads are complete rotations
        msg = ''
        on_left = False
        while True:
            while len(msg) < 6632:
                msg += self.socket.recv(4096)
            assert msg[:8] == preamble,\
                     "LIDAR status packet "+','.join((str(ord(char)) for char in msg[:8]))
            read, last_angle = _readFromLidar(msg)
            if on_left and last_angle > pi:
                break
            if not on_left and last_angle < pi:
                on_left = True
            msg = msg[6632:]
        self.leftovermsg = msg
            
        Process.start(self)
        self.time = time()
        return self
    
    def get(self, timeout=.01):
        try:
            points = self.queue.get(timeout=timeout)
            self.time = time()
        except Empty: # no rotation available, just wait
            assert time() - self.time < self.max_gap_time, "LIDAR not giving recent data"
            points = self.points
        try:
            while True:
                points = self.queue.get(block=False) # grab extra waiting rotations
        except Empty: pass
        self.gap_time = False
        self.points = points
        return points
    
    # this is the part that runs in a separate thread - or at least it should
    def run(self):
        rotation = []
        msg = self.leftovermsg
        on_left = False
        select_sock = (self.socket,)
        while True:
            readable,a,b = select.select(select_sock, [], [], .1)
            assert len(readable) > 0, "LIDAR rx timeout"
            while len(msg) < 6632:
                msg += self.socket.recv(4096)
                
            read, last_angle = _readFromLidar(msg)
            if on_left and last_angle > pi: # rotation complete
                on_left = False
                self.queue.put(tuple(rotation))
                rotation = []
            elif not on_left and last_angle < pi:
                on_left = True
            rotation += read
            
            msg = msg[6632:]
    
    def __exit__(self, errtype=None, errval=None, traceback=None):
#        if not (errtype is None or errtype is KeyboardInterrupt or
#                                   errtype is SystemExit):
#            print(errtype)
#            print(errval)
        self.terminate()
        
    def terminate(self):
        super(LIDAR, self).terminate()
        self.queue.close()
        self.socket.close()
        print("closed")
        
        
        
if __name__ == '__main__':
    # plot lidar points
    import numpy as np
    import cv2
    size = 320 # image size is 640x640
    base_image = np.zeros((size*2, size*2, 3), dtype=np.uint8) + 255
    base_image[size-5:size+5, size-5:size+5] = [230,230,230]
    img = base_image.copy()
    class Display():
        def __init__(self): pass
        def __enter__(self):
            cv2.imshow('lidar side detections', base_image)
            cv2.waitKey(5)
            return self
        def __exit__(self, a, b, c): cv2.destroyWindow('lidar side detections')
        def display(self, image):
            cv2.imshow('lidar side detections', image)
            cv2.waitKey(5)
            
    color = (0,0,0)
    
    offsets = ((-1,0),(1,0),(0,-1),(0,1),(0,0))
    
    with LIDAR() as lidar, Display() as display:
        while True:
            cloud = lidar.get()
            if cloud==():
                sleep(.1)
                continue
            
            cloud = np.array(cloud)
            cloud *= -size/max_dist_to_process
            cloud += size
            include = np.all((cloud>=0) & (cloud < size*2), axis=1)
            cloud = cloud[include].astype(int)
            img = base_image.copy()
#            img[:] = base_image
            for offx, offy in offsets:
                img[cloud[:,0]+offx, cloud[:,1]+offy] = color
            display.display(img)
            sleep(.1)
