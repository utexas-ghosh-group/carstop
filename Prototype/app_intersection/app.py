#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 1/10/19
uses multi-object tracking on input lidar data
takes input self-tracking data for motion reference

"""

import numpy as np
import numba as nb
from time import sleep
from prototype.app_intersection.occlusion_polar import visual_resolution, occludedByMap, doubleUp
from prototype.app_intersection.plotStuff import Display, DisplayNULL, size, plotRectangle, base_image, colorList
from prototype.app_intersection.options import real_vs_simulator

if real_vs_simulator:
    from prototype.sensors.quanergyBox import LIDAR
    from prototype.sensors.selfTrack import SelfTrack
else:
    from prototype.app_intersection.quanergyBoxFile import LIDAR
    from prototype.app_intersection.selfTrackRecorded import SelfTrack

import prototype.app_intersection.GStracker as GStracker
import prototype.app_intersection.boxTracker2 as bT

laser = 6

min_dist_xy = 1.
max_dist_xy = 40.

def debug(s):
    assert np.all(abs(s[:,1:3]) < 50)
    assert np.all(s[:,3:5] < 10)
    assert np.all(s[:,3:5] >= 0)

size_convert = size / max_dist_xy

# how to find that an object is past the laser's range?
# take vertical line segment, move to lidar reference frame
# check how close line segment is to lidar line
@nb.jit(nb.f8(nb.f8[:]), nopython=True)
def detect(sample):
    distance = np.hypot(sample[1],sample[2])
    radius = np.hypot(sample[3],sample[4])
    uncertain = np.sqrt(sample[7] + sample[14]-sample[8])/2.
    distance_in_min = max(distance + radius + uncertain - min_dist_xy, 0)
    min_distance_term = 1 - np.exp(-2*distance_in_min)
    distance_in_max = max(max_dist_xy - distance + radius + uncertain, 0)
    max_distance_term = 1 - np.exp(-2*distance_in_max)
    return .05 + .91 * min_distance_term * max_distance_term

# the most clockwise angle a measurement can have
# otherwise, calculating occlusion correctly
occluding_angle_tolerance = .005
@nb.jit(nb.f8(nb.f8[:],nb.f8[:,:]))
def occlude(sample, measurements_occluding):
                    
    mean = sample[:5]#[[1,2,0,3,4]]#.copy()
    deviation_center = np.sqrt(sample[7] + sample[14])/3. + .15
    current_distance = np.hypot(sample[1], sample[2])
    if deviation_center > current_distance: return 0.
    deviation_center = -deviation_center / current_distance
    deviation_len = np.sqrt(sample[21])/3. + .05
    deviation_wid = np.sqrt(sample[28])/3. + .05
    deviation = (0, deviation_center*sample[1], deviation_center*sample[2],
                 deviation_len, deviation_wid)
    
    rect0 = np.zeros(2)
    rect1 = np.zeros(2)
    mean2 = mean-deviation
    mean2[3] = max(mean2[3], .001)
    mean2[4] = max(mean2[4], .001)
    rectOccludeForm(mean2, rect0, rect1)
    if rect0[0] < 0 or rect1[0] < 0:
        vis_far = 1.
    else:
        vis_far = occludedByMap(rect0, rect1, measurements_occluding)
    if vis_far > .022: return 0.
    rectOccludeForm(mean, rect0, rect1)
    vis_mean = occludedByMap(rect0, rect1, measurements_occluding)
    if vis_mean > .022: return .4
    rectOccludeForm(mean+deviation, rect0, rect1)
    vis_near = occludedByMap(rect0, rect1, measurements_occluding)
    if vis_near > .022: return .8
    if vis_near > .01: return .95
    return .98

@nb.jit(nb.void(nb.f8[:], nb.f8[:], nb.f8[:]))
def rectOccludeForm(rect, output0, output1):
    angle,x,y,l,w = rect
    cos = np.cos(angle)
    sin = np.sin(angle)
    p0 = cos*x + sin*y
    p1 = sin*x - cos*y
    if abs(p0) <= l:
        if p1 > 0:
            x1 = -1
            y1 = 1
            x2 = 1
            y2 = 1
        else:
            x1 = 1
            y1 = -1
            x2 = -1
            y2 = -1
    elif abs(p1) <= w:
        if p0 < 0:
            x1 = 1
            y1 = 1
            x2 = 1
            y2 = -1
        else:
            x1 = -1
            y1 = -1
            x2 = -1
            y2 = 1
    elif p0 < 0:
        if p1 > 0:
            x1 = -1
            y1 = 1
            x2 = 1
            y2 = -1
        else:
            x1 = 1
            y1 = 1
            x2 = -1
            y2 = -1
    else:
        if p1 > 0:
            x1 = -1
            y1 = -1
            x2 = 1
            y2 = 1
        else:
            x1 = 1
            y1 = -1
            x2 = -1
            y2 = 1
    x1 *= l
    x2 *= l
    y1 *= w
    y2 *= w
    output0[0] = x1 * cos - y1 * sin + x
    output1[0] = x2 * cos - y2 * sin + x
    output0[1] = y1 * cos + x1 * sin + y
    output1[1] = y2 * cos + x2 * sin + y                 



position_motion_noise = .1 * .5**2
shape_motion_noise = .1 * .1**2
speed_motion_noise = .1 * 1.**2
@nb.jit(nb.void(nb.f8[:], nb.f8[:,:]), nopython=True)
def predict(sample, covs):
    sample[1] += sample[5]*.1
    sample[2] += sample[6]*.1
    bT.covAsSquare(sample, covs)
    covs[:2,:] += covs[4:,:]*.1
    covs[:,:2] += covs[:,4:]*.1
    covs[0,0] += position_motion_noise
    covs[1,1] += position_motion_noise
    covs[2,2] += shape_motion_noise
    covs[3,3] += shape_motion_noise
    covs[4,4] += speed_motion_noise
    covs[5,5] += speed_motion_noise
    
    covs += covs.T.copy()
    covs /= 2.
    eigvals = np.linalg.eigvalsh(covs)
    assert np.all(eigvals >= 0)
    bT.covAsList(sample, covs)

@nb.jit(nb.f8(nb.f8[:]), nopython=True)
def survival(sample):
    distance = np.hypot(sample[1],sample[2])
    radius = np.hypot(sample[3],sample[4])
    uncertain = np.sqrt(sample[7] + sample[14]-sample[8])/2.
    distance_in_min = max(distance + radius + uncertain - min_dist_xy, 0)
    min_distance_term = 1 - np.exp(-2*distance_in_min)
    distance_in_max = max(max_dist_xy - distance + radius + uncertain, 0)
    max_distance_term = 1 - np.exp(-2*distance_in_max)
    return .3 + .68 * min_distance_term * max_distance_term * (sample[1]>-3)

def distOnLineAtSlope(x1,y1,x2,y2, xc,yc):
    num = x1*y2 - y1*x2
    denom = xc*(y2-y1) + yc*(x1-x2)
    if denom == 0: # all the points are in a line
        return y1/yc if xc == 0 else x1/xc
    return num / denom

label_to_color = {}
n_colors = 100
colors = colorList(n_colors)

last_position = None

Self = GStracker.Tracker(100, 8, 43, 50)
Self.fnPredict = predict
Self.fnOcclude = occlude
Self.fnDetect = detect
Self.fnSurvival = survival
Self.fnPrepObject = bT.prepObject
Self.prep = bT.preps
Self.fnLikelihood = bT.likelihood
Self.fnUpdate = bT.update
Self.fnEntryFromMsmt = bT.sampleFromMsmt
Self.fnDebug = debug

Self.reset()
time = 0

with LIDAR() as lidar, Display() as display, SelfTrack() as selftrack:
    while True:
        
        # propagate up to this timestep
        Self.predict()
        # move self
        position = selftrack.get()
        pos_ang = position['heading']
        position = np.array([[np.cos(pos_ang), -np.sin(pos_ang), position['x']],
                             [np.sin(pos_ang),  np.cos(pos_ang), position['y']],
                             [0,0,1]])
        if last_position is not None:
            self_moved = np.linalg.inv(position).dot(last_position)
            Self.samples[:,1:3] = Self.samples[:,1:3].dot(self_moved[:2,:2].T)
            Self.samples[:,1:3] += self_moved[:2,2]
            Self.samples[:,0] += np.arctan2(self_moved[1,0],self_moved[0,0])
        last_position = position
        
        measurements, occlusion_map = lidar.get()
    
        if len(measurements) == 0:
            print("no msmts")
            continue
    
        new_measurements = []
        for msmt in measurements:
            assert msmt[0,0]*msmt[-1,1] - msmt[0,1]*msmt[-1,0] > -1e-3
            
            # ignore measurements that are not in the front semi-circle
            if np.max(msmt[:,0]) < 0: continue
            
            # for now, ignore merge dudes
            if msmt[1,3]: continue
            
            # pure planes (measurements with no corners) really don't work
            # without some kind of min/max bounding that you don't have
            if msmt.shape[0] == 2 and msmt[0,2]==0 and msmt[1,2]==0:
                msmt = msmt.copy()
                vector = msmt[1,:2]-msmt[0,:2]
                vector /= np.hypot(*vector)
                msmt[0,:2] -= vector*.2
                msmt[0,2] = True
            
            # new boxtracker takes measurements of fixed size
            if msmt.shape[0] == 2:
                msmt = np.append(msmt, np.zeros((1,4)), axis=0)
                msmt[2,2] = -1
        
            # have to do an after-the-fact fix here, for walls that turn the 'wrong' way
            goin = msmt[2,2] >= 0
            if goin:
                goin = np.hypot(msmt[1,0]-msmt[0,0],msmt[1,1]-msmt[0,1]) > .4
            if goin:
                goin = np.hypot(msmt[1,0]-msmt[2,0],msmt[1,1]-msmt[2,1]) > .4
            if goin:
                dB = distOnLineAtSlope(msmt[0,0],msmt[0,1],msmt[2,0],
                                       msmt[2,1],msmt[1,0],msmt[1,1])
                goin = dB < .99
            if goin:
                # split this segment into two
                msmt[1,2] = True
                first_changed_msmt = msmt.copy()
                first_changed_msmt[2,2] = -1
                second_changed_msmt = msmt.copy()
                second_changed_msmt[:2] = msmt[1:]
                second_changed_msmt[2,2] = -1
                new_measurements.append(first_changed_msmt)
                new_measurements.append(second_changed_msmt)
                continue
                
            new_measurements.append(msmt)
        measurements = new_measurements
            
        
        n_msmts = len(measurements)
        if n_msmts == 0:
            print("EMPTY")
            continue #raise Exception
        
        ## clean up occlusion map
        new_map = []
        point = occlusion_map[0]
        for nextpoint in occlusion_map[1:]:
            if nextpoint[0] - point[0] > visual_resolution:
                new_map.append(point)
                point = nextpoint
            else:
                point = (point[0], point[1], nextpoint[2])
        new_map.append(point)
        
        # make occlusion array
        measurements_occ = np.array(doubleUp(list(new_map)))
        measurements_occ[:,1:] += .25 # add distance, assuming that measurements won't be this close
        
        # decide probability that each measurement is a new object/false measurement
        if time == 0:
            entry_cardinality_sides = .1
            entry_cardinality_middle = .1
        else:
            entry_cardinality_sides = .025
            entry_cardinality_middle = .002
        entry_cardinalities = np.empty((n_msmts,))
        fp_msmt_probs = np.empty((n_msmts,))
        for msmt_idx in xrange(n_msmts):
            msmt = measurements[msmt_idx]
            
            distance = np.min(np.hypot(msmt[:,1],msmt[:,0]))
            entry_cardinality = entry_cardinality_sides if\
                    distance > max_dist_xy-1.5 else entry_cardinality_middle

            entry_cardinalities[msmt_idx] = entry_cardinality
            fp_msmt_probs[msmt_idx] = .05        
        
        # let's understand the merge situation
        merge_situations = []
        found_ids = np.zeros(30, dtype=bool)
        not_in_merges = []
        for k in xrange(n_msmts):
            situation_id = int(measurements[k][0,3])
            if True:#situation_id == 0:
                not_in_merges.append(k)
            elif found_ids[situation_id-1]:
                merge_situation = merge_situations[situation_id-1]
                if measurements[k][1,3]:
                    assert merge_situation[0] is None
                    merge_situation[0] = k
                else:
                    merge_situation[1].append(k)
            else:
                if measurements[k][1,3]:
                    merge_situation = [k, []]
                else:
                    merge_situation = [None, [k]]
                merge_situations.append(merge_situation)
                found_ids[situation_id-1] = True
        measurements = [msm2t[:,:3].copy() for msm2t in measurements] # just in case
        assert all(background is not None for background, foreground in merge_situations)
        assert all(len(foreground) > 1 and len(foreground) < 5
                       for background, foreground in merge_situations)
        assert len(merge_situations) < 4
        print("{:d} merges".format(len(merge_situations)))
        
        # inclusion matrices for each merge possibility
        # 2^(number of merges)
        n_situations = 2**len(merge_situations)
        merge_include = []
        for situation_id in xrange(n_situations):
            merge_include_sit = list(not_in_merges)
            for merge_id, merge_situation in enumerate(merge_situations):
                if situation_id % 2 > 0:
                    merge_include_sit.append(merge_situation[0])
                else:
                    merge_include_sit += merge_situation[1]
                situation_id = situation_id // 2
            merge_include.append(merge_include_sit)
        assert len(merge_include) > 0
        assert len(measurements) > 0
        
        
        # update and clean
        Self.update(measurements, fp_msmt_probs, entry_cardinalities,
                    measurements_occ, merge_include)
        Self.debug()
        Self.resolveUpdate()
        assert Self.nhyp > 0
            
        if time % 4 == 0:
            Self.prune()
        time += 1
                    
        ## report and plot
        output = Self.report(bT.report, cutoff=.5)
        img = base_image.copy()
        
        for out in output:
            
            # figure out color for each object
            label = out[0]
            if label in label_to_color:
                color = colors[label_to_color[label]]
            else:
                new_color = int(np.random.rand() * n_colors)
                label_to_color[label] = new_color
                color = colors[new_color]
            
            rectangle = out[2:].copy()
            rectangle[3:5] = np.maximum(rectangle[3:5], .1)
            rectangle[[0,1,3,4]] *= size_convert
            
            plotRectangle(img, rectangle, color)

        gg = np.array([corner[:2] for msm2t in measurements for corner in msm2t])
        gg *= -size_convert
        gg += size
        gg = gg.astype(int)
        gg_include = gg[:,0] >= 0
        gg_include &= gg[:,1] >= 0
        gg_include = gg[:,0] < size*2
        gg_include = gg[:,1] < size*2
        gg = gg[gg_include]
        for offx, offy in [(1,0),(-1,0),(0,1),(0,-1),(0,0),(-1,-1),(1,1),(-1,1),(1,-1)]:
            img[gg[:,0]+offx,gg[:,1]+offy] = 0

        display.display(img)
        
        sleep(.05)