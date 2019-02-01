#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 11/8/18
"""

from math import cos, sin, tan, pi
import socket, select, struct
from multiprocessing import Process, Queue#, Manager
from Queue import Empty
from time import time

from multiprocessing import freeze_support
if __name__ == '__main__':
    freeze_support()

laser = 5
min_dist_to_process = 1. # this is the lowest distance that the lidar returns
max_dist_xy = 10.
max_dist_to_process = max_dist_xy * 2.**.5

preamble = ''.join((chr(int(byte, 16)) for byte in ['75','bd','7e','97']))
preamble += struct.pack(">L", 6632)
firingidxlist = [firing*132 for firing in range(50)]
fullpackedstructure = ">H8L"
angle_conversion = pi / 5200.
distance_conversion = 1e-5
vertical_angles = (-.318505,-.2692,-.218009,-.165195,-.111003,
                            -.0557982,0.,.0557982)
vertical_angle = vertical_angles[laser]
vertical_component = sin(vertical_angle) * distance_conversion
horizontal_component = cos(vertical_angle) * distance_conversion
vertical_transform = tan(vertical_angle)


class LIDAR(Process):
    def __init__(self, IP='10.1.11.129', port=4141, max_gap_time = .3):
        Process.__init__(self)
        self.IP = IP
        self.port = port
        self.queue = Queue()
        self.time = 0.
        self.max_gap_time = max_gap_time
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        
    def __enter__(self):
        print("connecting to LIDAR...")
        self.socket.connect((self.IP, self.port))
        print "LIDAR connected"
        print("waiting for first LIDAR packet...")
        ready_to_read, ready_to_write, in_error = select.select(
               (self.socket,), [], [], 30.)
        assert len(ready_to_read) > 0, "LIDAR not running within 30 seconds"
        print("LIDAR sending packets")
            
        # run through a rotation, so you start with full rotations
        msg = ''
        on_left = False
        while True:
            while len(msg) < 6632:
                msg += self.socket.recv(4096)
            assert msg[:8] == preamble,\
                     "LIDAR status packet "+','.join((str(ord(char)) for char in msg[:8]))
            
            for i in firingidxlist:
                firinglist = struct.unpack(fullpackedstructure,
                                           msg[i+20:i+22]+msg[i+24:i+56])
                angle = firinglist[0] * angle_conversion
            msg = msg[6632:]
            
            if on_left and angle > pi:
                break
            if not on_left and angle < pi:
                on_left = True
            
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
    
    
    # this is the part that runs in a separate thread
    def run(self):
        msg = self.leftovermsg
        on_left = True
        rotation = []
        select_sock = (self.socket,)
        while True:
            readable,a,b = select.select(select_sock, [], [], .1)
            assert len(readable) > 0, "LIDAR rx timeout"
            while len(msg) < 6632:
                msg += self.socket.recv(4096)
            assert msg[:8] == preamble,\
                     "LIDAR status packet "+','.join((str(ord(char)) for char in msg[:8]))
            
            for i in firingidxlist:
                firinglist = struct.unpack(fullpackedstructure,
                                           msg[i+20:i+22]+msg[i+24:i+56])
                angle = firinglist[0] * angle_conversion
                dist = firinglist[1+laser] * horizontal_component
                if dist > min_dist_to_process and dist < max_dist_to_process:
                    x = dist * cos(angle)
                    y = dist * sin(angle)
                    #z = firinglist[1+laser] * vertical_component
                    rotation.append((x,y))
            
            if on_left and angle > pi:
                # crossed to the right in the back, store info now
                on_left = False
                self.queue.put(tuple(rotation))
                rotation = []
            elif not on_left and angle < pi:
                on_left = True
            msg = msg[6632:]
    
    def __exit__(self, errtype=None, errval=None, traceback=None):
        if not (errtype is None or errtype is KeyboardInterrupt or
                                   errtype is SystemExit):
            print(errtype)
            print(errval)
        self.terminate()
        
    def terminate(self):
        self.queue.close()
        super(LIDAR, self).terminate()
        self.socket.close()
        print("closed")
        
        
        
        
if __name__ == '__main__':
    import numpy as np
    import cv2
    size = 320
    base_image = np.zeros((size*2, size*2, 3), dtype=np.uint8) + 255
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
            
    color = (0,0,200)
    
#    # square
#    size = 1
#    offsx = range(-size,size+1)
#    offsy = range(-size,size+1)
#    offsets = tuple([(offx, offy) for offx in offsx for offy in offsy])
    # cross
    offsets = ((-1,0),(1,0),(0,-1),(0,1),(0,0))
    
    with LIDAR() as lidar, Display() as display:
        while True:
            cloud = lidar.get(.1)
            if len(cloud) ==0: continue
            
            cloud = np.array(cloud)
            cloud *= -size/max_dist_xy
            cloud += size
            include = np.all((cloud>=1) & (cloud < size*2-1), axis=1)
            cloud = cloud[include].astype(int)
            img = base_image.copy()
            for offx, offy in offsets:
                img[cloud[:,0]+offx, cloud[:,1]+offy] = color
            display.display(img)
