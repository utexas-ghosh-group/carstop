#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 11/3/18
returns points from numpy-saved file, applies box tracking to them
"""
import os
import numpy as np
from math import atan2, hypot

import prototype.app_intersection.point2rect as point2rect
from prototype.app_intersection.options import recorded_lidar_folder as name

def distOnLineAtSlope(x1,y1,x2,y2, xc,yc):
    num = x1*y2 - y1*x2
    denom = xc*(y2-y1) + yc*(x1-x2)
    if denom == 0: # all the points are in a line
        return y1/yc if xc == 0 else x1/xc
    return num / denom

class LIDAR():
    def __init__(self):
        self.files = sorted(os.listdir(name))
        self.fileidx = 1
        self.lastdata = np.load(name + self.files[0])
        lastdata_break = np.where(np.diff(np.arctan2(self.lastdata[:,1],self.lastdata[:,0]))<0)[0]
        assert len(lastdata_break)==1
        lastdata_break = lastdata_break[0]
        self.lastdata = self.lastdata[lastdata_break:]
        
    def __enter__(self): return self
    
    def get(self):
        if self.fileidx >= len(self.files):
            raise StopIteration
        data = np.load(name + self.files[self.fileidx])
        self.fileidx += 1
        
        data_break = np.where(np.diff(np.arctan2(data[:,1],data[:,0]))<-.2)[0]
        if len(data_break) == 1:
            data_break = data_break[0]
            new_data = np.concatenate((self.lastdata, data[:data_break]), axis=0)
            self.lastdata = data[data_break:]
            data = new_data
        elif len(data_break) == 0:
            self.lastdata = np.zeros((0,2))
        else:
            np.save('offender.npy', data)
            raise Exception
        
        #data = np.load(name + lidar.files[2])
        segments = []
        occlusion_map = []
        points = []
        merge_count = 1
        
        for x,y in data[::2]:
            new_segment = point2rect.addPoint((x,y), points)
            if new_segment is not None:
                merge_count = point2rect.segmentPoints(new_segment,
                                segments, merge_count, occlusion_map)
                
        if len(segments) > 0:
            # add/fix last occlusion
            last_edge = segments[-1][-1]
            last_occlusion = (atan2(last_edge[1],last_edge[0]),
                              hypot(last_edge[1],last_edge[0]), 1e10)
            if last_occlusion[0] > 0:
                if last_occlusion[0] > occlusion_map[-1][0]:
                    occlusion_map.append(last_occlusion)
            else:
                if last_occlusion[0] < occlusion_map[0][0]:
                    occlusion_map.insert(0, last_occlusion)
        # fix first occlusion
        if len(occlusion_map) > 0 and occlusion_map[0][0] > 0:
                first_occlusion = occlusion_map.pop(0)
                if first_occlusion[0] >= occlusion_map[-1][0]:
                    occlusion_map.append(first_occlusion)
                else:
                    print("no first occlusion")
                    
        # have to do an after-the-fact fix here, for walls that turn the 'wrong' way
        for seg_idx, seg in enumerate(segments):
            assert seg.shape[0] >= 2
            if seg.shape[0] == 2: continue
            if hypot(seg[2,0]-seg[0,0], seg[2,1]-seg[0,1]) < .5: continue
            dB = distOnLineAtSlope(seg[0,0],seg[0,1],seg[2,0],seg[2,1],seg[1,0],seg[1,1])
            if dB < 1:
                # split this segment into two
                seg[1,2] = True
                if seg[0,0]*seg[1,1]-seg[0,1]*seg[1,0] > 0:
                    segments[seg_idx] = seg[:2].copy()
                else:
                    segments[seg_idx] = seg[1::-1].copy()
                if seg[1,0]*seg[2,1]-seg[1,1]*seg[2,0] > 0:
                    segments.append(seg[1:].copy())
                else:
                    segments.append(seg[2:0:-1].copy())
                    
        return tuple(segments), tuple(occlusion_map)
    
    def __exit__(self, errtype, errval, traceback): pass






if __name__ == '__main__':
    max_dist_xy = 40.
    from occlusion_polar import visual_resolution
    from plotStuff import size, base_image, fillArc, drawLine, Display
    import time
#    from cv2 import imwrite
    img = base_image.copy()
            
    color_back = (110,110,210)
    color_normal = (0,0,0)
    color_occlusion = (240,240,240)
    
    size_convert = size / max_dist_xy
    
    
    
    with LIDAR() as lidar, Display() as display:
        kkk = -1
        while True:
            time.sleep(.2)
            kkk+=1
            measurements, occlusion_map = lidar.get()
            img = base_image.copy()# img[:] = base_image
            
            if len(measurements) == 0 or len(occlusion_map) == 0:
                print("empty")
                continue
            
            ms2 = []
            for msmt in measurements:
                if msmt.shape[0] == 0: continue
                assert msmt[0,0]*msmt[-1,1] - msmt[0,1]*msmt[-1,0] > -1e-3
#                # for now, ignore merge dudes
#                if msmt[1,3]: continue
            
#                # for desk area, ignore back (window)
#                if np.all(msmt[:,0] < 0) and np.all(abs(msmt[:,1])<msmt[:,0]*-.4):
#                    continue
                ms2.append(msmt)
            measurements = ms2
            
            
            # prune occlusion
            new_map = []
            point = occlusion_map[0]
            for nextpoint in occlusion_map[1:]:
                if nextpoint[0] - point[0] > visual_resolution:
                    new_map.append(point)
                    point = nextpoint
                else:
                    point = (point[0], point[1], nextpoint[2])
            new_map.append(point)
            old_map = occlusion_map
            occlusion_map = new_map
            
            point = occlusion_map[0]
            for nextpoint in occlusion_map[1:]:
                if nextpoint[0]-point[0] <= 0:
                    print("map error! angle {:d}".format(kkk))
                    break
                if nextpoint[1]>1000 and point[2]<1000:
                    print("map error! dist {:d}".format(kkk))
                    break
                point = nextpoint
            
            # plot occlusion
            for j in range(len(occlusion_map)-1):
                angle1,_,d1 = occlusion_map[j]
                angle2,d2,_ = occlusion_map[j+1]
                if (angle2-angle1)%(2*np.pi) >= np.pi:
                    print("angle problem {:f}, {:f}".format(angle1, angle2))
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
                msmt_points_x = np.array(msmt_x_back, dtype=int)
                msmt_points_y = np.array(msmt_y_back, dtype=int)
                include_msmt_points = (msmt_points_x >= 1) & (msmt_points_y > 1) &\
                            (msmt_points_x < size*2-1) & (msmt_points_y < size*2-1)
                msmt_points_x = msmt_points_x[include_msmt_points]
                msmt_points_y = msmt_points_y[include_msmt_points]
                for offset_x in range(-1,1):
                    for offset_y in range(-1,1):
                        img[msmt_points_x+offset_x, msmt_points_y+offset_y] = color_back
                msmt_points_x = np.array(msmt_x_normal, dtype=int)
                msmt_points_y = np.array(msmt_y_normal, dtype=int)
                include_msmt_points = (msmt_points_x >= 1) & (msmt_points_y > 1) &\
                            (msmt_points_x < size*2-1) & (msmt_points_y < size*2-1)
                msmt_points_x = msmt_points_x[include_msmt_points]
                msmt_points_y = msmt_points_y[include_msmt_points]
                for offset_x in range(-1,1):
                    for offset_y in range(-1,1):
                        img[msmt_points_x+offset_x, msmt_points_y+offset_y] = color_normal
            
            display.display(img)
#            if kkk == 0:
#                imwrite('whiteboard6.png', img)
#                break
