#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 8/13/18
computer's IP must be at 192.168.1.0 (or whatever lidar is configured to send to)

function = get(block, timeout)
if behind, can read up to current point with
while not lidar.empty(): info = lidar.get(False)

output = numpy float array, variable length, width 3
         xyz of each laser point
"""
import numpy as np
import socket, select, struct
from multiprocessing import Process, Queue#, Manager

distance_convert = .002
vert_angles = [-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15]
vert_cos = [np.cos(vert_angle*np.pi/180)*distance_convert for vert_angle in vert_angles]
vert_sin = [np.sin(vert_angle*np.pi/180)*distance_convert for vert_angle in vert_angles]
vert_sin = vert_sin * 2 # two firings at once
shortStruct = struct.Struct('<H')
longStruct = struct.Struct('<L')
firefmt = struct.Struct('<'+ 'HB'*32)
angle_convert = np.pi / 18000.
time_convert = 1e-6
angle_step = .0033 # radians of offset firing
step_cos = np.cos(angle_step)
step_sin = np.sin(angle_step)

min_distance_to_keep = 2. / distance_convert
max_distance_to_keep = 100. / distance_convert

def processLidarPacket(msg, last_angle = 0):
    xyz = np.empty((384,3))
    keep = np.empty((384,),dtype=bool)
    cut = False
    cut_idx = 384
    
    for k in range(12):
        angle = shortStruct.unpack(msg[k*100 + 2: k*100 + 4])[0] * angle_convert
        hcos = np.cos(angle)
        hsin = np.sin(angle)
        hcos2 = hcos * step_cos - hsin * step_sin
        hsin2 = hcos * step_sin + hsin * step_cos
        if angle < last_angle:
            cut = True
            cut_idx = k*32
        last_angle = angle
        
        #fires = np.array(firefmt.unpack(msg[k*100 + 4: k*100 + 100])).reshape((32,2))
        fires = np.array(firefmt.unpack(msg[k*100 + 4 : k*100 + 100])[::2])
        xyz[k*32:k*32+16,0] = fires[:16] * hcos * vert_cos
        xyz[k*32+16:k*32+32,0] = fires[16:32] * hcos2 * vert_cos
        xyz[k*32:k*32+16,1] = fires[:16] * -hsin * vert_cos
        xyz[k*32+16:k*32+32,1] = fires[16:32] * -hsin2 * vert_cos
        xyz[k*32:k*32+32,2] = fires * vert_sin
        keep[k*32:k*32+32] = (fires > min_distance_to_keep) &\
                             (fires < max_distance_to_keep)
        
    time = longStruct.unpack(msg[1200:1204])[0] * time_convert
    return xyz[:cut_idx][keep[:cut_idx]], cut, xyz[cut_idx:][keep[cut_idx:]], time, angle


class LIDAR(Process):
    def __init__(self, queue=None, port=2368):
        Process.__init__(self)
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.port = port
        if queue is None:
            self.queue = Queue()
        else:
            self.queue = queue
        
    def __enter__(self):
        print("connecting to LIDAR...")
        self.sock.bind(('', self.port))
        print("waiting for first LIDAR packet...")
        self.select_sock = (self.sock,)
        ready_to_read, to_write, in_error = select.select(self.select_sock, [], [], 30.)
        assert len(ready_to_read) > 0, "LIDAR not running within 30 seconds"
        print("LIDAR working")
        
        # do one read so that all future reads are complete rotations
        msg = self.sock.recv(1206)
        assert len(msg) == 1206
        data_pre, cut, data_post, time, angle = processLidarPacket(msg, 0.)
        while not cut:
            readable,a,b = select.select(self.select_sock, [], [], .1)
            assert len(readable) > 0, "LIDAR rx timeout"
            msg = self.sock.recv(1206)
            assert len(msg) == 1206
            data_pre, cut, data_post, time, angle = processLidarPacket(msg, angle)
        
        # start stuff
        Process.start(self)
        return self.queue
        
    # this is the part that runs in a separate thread - or at least it should
    def run(self):
        angle = 0
        rotation = []
        while True:
            readable,a,b = select.select(self.select_sock, [], [], .1)
            assert len(readable) > 0, "LIDAR rx timeout"
            msg = self.sock.recv(1206)
            assert len(msg) == 1206
            data_pre, cut, data_post, time, angle = processLidarPacket(msg, angle)
            rotation.append(data_pre)
            if cut:
                self.queue.put(np.concatenate(rotation, axis=0))
                rotation = [data_post]
    
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
        
        
### test
if __name__ == '__main__':
    
    # set up real-time plot
    import matplotlib.pyplot as plt
    from matplotlib.cm import ScalarMappable
    from matplotlib.colors import Normalize
    fig1 = plt.figure(figsize=(8., 8.))
    fig1.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=None, hspace=None)
    ax = fig1.gca()
    ax.set_xlim([-50,50])
    ax.set_ylim([-50,50])
    plt.margins(0,0)
    mycar = ax.add_patch(plt.Rectangle((-3.6,-1),5.,2.,0.,fill=True, color='k'))
    scatterpoints = ax.scatter([], [], 2., color=[],
                                 cmap = ScalarMappable(Normalize(0,7)))
    fig1.canvas.draw()
    plt.show(block=False)
    fig1.canvas.update()
    fig1.canvas.flush_events()
    rotation_period = 1 # optionally don't update plot every time
    rotations = 0
    

    with LIDAR() as lidar:
        while True:
            data = lidar.get(timeout=.11)
            
            # update plot
            rotations += 1
            if rotations % rotation_period == 0:
#                thistime = time.time()
#                print thistime - lasttime
#                lasttime = thistime
                # get downward points
                data = data[data[:,2] < 0]
                # get points past car itself
                data = data[np.hypot(data[:,0], data[:,1]) > 1.5]
                scatterpoints.set_offsets(data[:,:2])
                scatterpoints.set_array(data[:,2])
                ax.draw_artist(ax.patch)
                ax.draw_artist(mycar)
                ax.draw_artist(scatterpoints)
                fig1.canvas.update()
                fig1.canvas.flush_events()
