#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 11/1/18
"""

from math import cos, sin, tan, pi, atan2, hypot
import socket, select, struct
from multiprocessing import Process, Queue#, Manager
from Queue import Empty
import point2rect


laser = 6
min_dist_to_process = .3
max_dist_xy = 10.
max_dist_to_process = max_dist_xy * 2.**.5

preamble = ''.join((chr(int(byte, 16)) for byte in ['75','bd','7e','97']))
preamble += struct.pack(">L", 6632)
firingidxlist = [firing*132 for firing in range(50)]
fullpackedstructure = ">H8L"
angle_conversion = pi / 5200.
distance_conversion = 10.**-5
vertical_angles = (-.318505,-.2692,-.218009,-.165195,-.111003,
                            -.0557982,0.,.0557982)
vertical_angle = vertical_angles[laser]
vertical_component = sin(vertical_angle) * distance_conversion
horizontal_component = cos(vertical_angle) * distance_conversion
vertical_transform = tan(vertical_angle)


class LIDAR(Process):
    def __init__(self, IP='10.1.11.129', port=4141):
        Process.__init__(self)
        self.IP = IP
        self.port = port
        self.queue = Queue()
        self.gap_time = True
        #self.first_rotation = True
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.select_sock = (self.socket,)
        
    def __enter__(self):
        print("connecting to LIDAR...")
        self.socket.connect((self.IP, self.port))
        print "LIDAR connected"
        print("waiting for first LIDAR packet...")
        ready_to_read, ready_to_write, in_error = select.select(
               self.select_sock, [], [], 30.)
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
        return self
    
    def get(self, timeout=.15):
        try:
            points = self.queue.get(timeout=timeout)
        except Empty: # no rotation available, just wait
            assert not self.gap_time, "LIDAR not recent enough data"
            self.gap_time = True
            return self.points
        try:
            points = self.queue.get(block=False) # grab extra waiting rotations
        except Empty: pass
        self.gap_time = False
        self.points = points
        return points
    
    
    # this is the part that runs in a separate thread
    def run(self):
        segments = []
        occlusion_map = []
        points = []
        merge_count = 1
        msg = self.leftovermsg
        on_left = True
        while True:
            readable,a,b = select.select(self.select_sock, [], [], .1)
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
                    new_segment = point2rect.addPoint((x,y), points)
                
                    if new_segment is not None:
                        merge_count = point2rect.segmentPoints(new_segment,
                                        segments, merge_count, occlusion_map)
            
            if on_left and angle > pi:
                # crossed to the right in the back, store info now
                on_left = False
                # fix first occlusion
                if len(occlusion_map) > 0:
                    if occlusion_map[0][0] > 0:
                        occlusion_map.append(occlusion_map.pop(0))
                # add/fix last occlusion
                if len(segments) > 0:
                    last_edge = segments[-1][-1]
                    last_occlusion = (atan2(last_edge[1],last_edge[0]),
                                      hypot(last_edge[1],last_edge[0]), 1e10)
                    if last_occlusion[0] > 0:
                        if last_occlusion[0] > occlusion_map[-1][0]:
                            occlusion_map.append(last_occlusion)
                    else:
                        if last_occlusion[0] < occlusion_map[0][0]:
                            occlusion_map.insert(0, last_occlusion)
#                if len(segments) > 0:
#                    last_edge = segments[-1][-1]
#                    occlusion_map.append((atan2(last_edge[1],last_edge[0]),
#                                          hypot(last_edge[1],last_edge[0]),
#                                          1e10))
                self.queue.put((tuple(segments), tuple(occlusion_map)))
                segments = []
                occlusion_map = []
                merge_count = 1
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
        self.socket.close()
        self.queue.close()
        super(LIDAR, self).terminate()
        
        
        
        
        
if __name__ == '__main__':
    import numpy as np
    from plotStuff import size, base_image, fillArc, drawLine, Display
    img = base_image.copy()
            
    color_back = (110,110,210)
    color_normal = (0,0,0)
    color_occlusion = (240,240,240)
    
    size_convert = size / max_dist_xy
    
    
    
    with LIDAR() as lidar, Display() as display:
        while True:
            measurements, occlusion_map = lidar.get()
            img = base_image.copy()# img[:] = base_image
            
            if len(measurements) == 0 or len(occlusion_map) == 0:
                print("empty")
                continue
            
            # prune occlusion
            new_map = []
            point = occlusion_map[0]
            twopi = 2*pi
            for nextpoint in occlusion_map[1:]:
                if nextpoint[0] - point[0] > .005:
                    new_map.append(point)
                    point = nextpoint
                elif nextpoint[0] - point[0] > 0:
                    point = (point[0], point[1], nextpoint[2])
                else:
                    print("ewerer")
            new_map.append(point)
            old_map = occlusion_map
            occlusion_map = new_map
#            if occlusion_map[0][0] > 0:
#                occlusion_map = occlusion_map[1:] + occlusion_map[:1]
#            elif occlusion_map[-1][0] < 0:
#                occlusion_map = occlusion_map[-1:] + occlusion_map[:-1]
            
            point = occlusion_map[0]
            for nextpoint in occlusion_map[1:]:
                if nextpoint[0]-point[0] <= 0:
                    print("map error! angle")
                    break
                if nextpoint[1]>1000 and point[2]<1000:
                    print("map error! dist")
                    break
                point = nextpoint
            
            # plot occlusion
            for j in range(len(occlusion_map)-1):
                angle1,_,d1 = occlusion_map[j]
                angle2,d2,_ = occlusion_map[j+1]
                if (angle2-angle1)%(2*np.pi) >= np.pi:
#                    print("angle problem {:f}, {:f}".format(angle1, angle2))
                    continue
                if d1 < max_dist_xy and d2 < max_dist_xy:
                    d1 *= size_convert
                    d2 *= size_convert
                    fillArc(img, d1, d2, angle1, angle2, color_occlusion)

            # count number of merges
            n_merges = max(measurement[0,3] for measurement in measurements)
#            print("{:.0f} merges".format(n_merges))
            
            # plot measurements
            if len(measurements) > 0:
                msmt_x_normal = []
                msmt_y_normal = []
                msmt_x_back = []
                msmt_y_back = []
                for measurement in measurements:
                        for msmt_point_idx in range(len(measurement)-1):
                            m1x,m1y = measurement[msmt_point_idx,:2]
                            m2x,m2y = measurement[msmt_point_idx+1,:2]
                            px, py = drawLine(size-m1x*size_convert,
                                              size-m1y*size_convert,
                                              size-m2x*size_convert,
                                              size-m2y*size_convert)
                            if measurement[1,3]:
                                msmt_x_back += px
                                msmt_y_back += py
                            else:
                                msmt_x_normal += px
                                msmt_y_normal += py
                msmt_points_x = np.array(msmt_x_normal, dtype=int)
                msmt_points_y = np.array(msmt_y_normal, dtype=int)
                include_msmt_points = (msmt_points_x >= 1) & (msmt_points_y > 1) &\
                            (msmt_points_x < size*2-1) & (msmt_points_y < size*2-1)
                msmt_points_x = msmt_points_x[include_msmt_points]
                msmt_points_y = msmt_points_y[include_msmt_points]
                for offset_x in range(-1,1):
                    for offset_y in range(-1,1):
                        img[msmt_points_x+offset_x, msmt_points_y+offset_y] = color_normal
                msmt_points_x = np.array(msmt_x_back, dtype=int)
                msmt_points_y = np.array(msmt_y_back, dtype=int)
                include_msmt_points = (msmt_points_x >= 1) & (msmt_points_y > 1) &\
                            (msmt_points_x < size*2-1) & (msmt_points_y < size*2-1)
                msmt_points_x = msmt_points_x[include_msmt_points]
                msmt_points_y = msmt_points_y[include_msmt_points]
                for offset_x in range(-1,1):
                    for offset_y in range(-1,1):
                        img[msmt_points_x+offset_x, msmt_points_y+offset_y] = color_back
            
            display.display(img)