#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 4/27/18 measurement-wise occlusion with independent edge noise
obj = [level, pos, length, speed]
"""

import numpy as np
from skvideo.io import FFmpegWriter as vwriter

n_levels_left = 2
n_levels_right = 2
n_levels = n_levels_left + n_levels_right
road_len = 30. # in two directions
crossing_len = 3.
min_car_len = 2.
max_car_len = 7.
min_car_speed = 2. # at 5 fps, 10 m/s -> 22 mph
max_car_speed = 5. # at 5 fps, 25 m/s -> 56 mph
speed_walk_std = 0.

def drawBox2D(hmin,wmin,hmax,wmax, image, color=[40,40,150], linewidth=1,
              sides_to_include=[True,True,True,True]):
    lw = linewidth
    hmin = max(lw, int(hmin))
    hmax = min(image.shape[0] - lw, int(hmax))
    wmin = max(lw, int(wmin))
    wmax = min(image.shape[1] - lw, int(wmax))
    if sides_to_include[0]: # top
        image[hmin-lw:hmin+lw+1, wmin-lw:wmax+lw+1] = color
    if sides_to_include[1]: # left
        image[hmin-lw:hmax+lw+1, wmin-lw:wmin+lw+1] = color
    if sides_to_include[2]: # bottom
        image[hmax-lw:hmax+lw+1, wmin-lw:wmax+lw+1] = color
    if sides_to_include[3]: # right
        image[hmin-lw:hmax+lw+1, wmax-lw:wmax+lw+1] = color

def getLR(pos, length, speed):
    if type(pos) == np.ndarray:
        l = pos.copy()
        r = pos.copy()
        l[speed > 0] -= length[speed > 0]
        r[speed < 0] += length[speed < 0]
        return l,r
    else:
        if speed > 0:
            return pos - length, pos
        else:
            return pos, pos + length


# don't hit current object
# speed = random walk from old speed, make situation more challenging on average
def genNextObject(obj, rng):
    old_speed = np.abs(obj[3])
    max_speed = min(old_speed + 1, max_car_speed)
    min_speed = max(old_speed - 1, min_car_speed)
    sign = 1 if obj[0] < n_levels_left else -1
    speed = rng.uniform(min_speed, max_speed)
    if speed > obj[3]:
        old_time_to_pass = (road_len*2+obj[2])/old_speed
        new_time_to_pass = road_len*2/speed
        min_time = max(0.1, old_time_to_pass - new_time_to_pass)
    else:
        min_time = 0.1
    time = rng.uniform(min_time, 15) # 13 minimum
    #time = int(time) + 1 # round up
    length = rng.uniform(min_car_len, max_car_len)
    return time, [obj[0], -sign*road_len, length, sign*speed]
#speed_mean = 3.5
#speed_std = .76 # from simulation
#birth_rate = .105

def update(present_objects, planned_objects, rng=np.random):
    new_present_objects = []
    new_planned_objects = []
    n_new_objects = 0
    for obj in present_objects:
        obj[3] += rng.normal(scale=speed_walk_std)
        obj[1] += obj[3]
        if np.abs(obj[1]) < road_len + obj[2]:
            new_present_objects.append(obj)
    for entry_time, obj in planned_objects:
        if entry_time <= 0:
            obj[1] = obj[1] - obj[3]*entry_time # for time discretization
            new_present_objects.append(obj)
            entry_gap, new_obj = genNextObject(obj, rng)
            new_planned_objects.append((entry_gap+entry_time,new_obj))
            n_new_objects += 1
        else:
            new_planned_objects.append((entry_time - 1, obj))
    return new_present_objects, new_planned_objects, n_new_objects

def isCollision(objects):
    for i in range(len(objects)):
        for j in range(i+1,len(objects)):
            if objects[i][0] == objects[j][0]:
                lr_i = getLR(*objects[i][1:])
                lr_j = getLR(*objects[j][1:])
                if lr_j[1] > lr_i[0] and lr_i[1] > lr_j[0]:
                    return True
    return False

#sense_points = np.linspace(-road_len, road_len, 16, endpoint=True)
#sense_point_range = .5
#sense_err = .24 # probability that msmt_1 is correct
#sense_n = .5
#def sense(objects):
#    detection_levels = np.zeros(sense_points.shape, dtype=int)
#    detection_pos = np.zeros(sense_points.shape)
#    for k, sensor_pos in enumerate(sense_points):
#        real_pos = sensor_pos + np.random.uniform(-1.,1)*sense_point_range
#        
#        for level, pos, length, speed in objects:

    
fp_rate = 0.15
detect_prob = .99
pos_noise_std = 0.75
def sense_OWO(objects, rng=np.random):
    detections = []
    # determine which objects (and which parts of objects) are visible
    edges = [] # flip the openness of the space
    for level in range(n_levels):
        cross_len = crossing_len * (level+1)
        for obj_level, pos, length, speed in objects:
            if obj_level != level: continue
            left_pos, right_pos = getLR(pos, length, speed)
            left_angle = left_pos/cross_len
            right_angle = right_pos/cross_len
            left_edge, right_edge = np.searchsorted(edges, [left_angle, right_angle])
            new_edges = []
            edge_visible_left = left_edge%2==0 # open space
            if left_edge == right_edge and not edge_visible_left:
                continue # full occlusion
            if edge_visible_left:
                edge_output_left = [True, left_pos, left_pos]
                new_edges += [left_angle]
            else:
                edge_output_left = [False, edges[left_edge-1]*cross_len,
                                           edges[left_edge]*cross_len]
            edge_visible_right = right_edge%2==0
            if edge_visible_right:
                edge_output_right = [True, right_pos, right_pos]
                new_edges += [right_angle]
            else:
                edge_output_right = [False, edges[right_edge-1]*cross_len,
                                           edges[right_edge]*cross_len]
            detections += [[level]+edge_output_left+edge_output_right]
            edges = edges[:left_edge] + new_edges + edges[right_edge:]
    detections = np.array(detections, dtype=float)
    
    # false negatives
    keep_detections = rng.rand(detections.shape[0]) < detect_prob
    detections = detections[keep_detections]
    
    # noise
    left_noise = rng.normal(scale=pos_noise_std, size=detections.shape[0])
    detections[:,2] += left_noise
    detections[:,3] += left_noise
    right_noise = rng.normal(scale=pos_noise_std, size=detections.shape[0])
    detections[:,5] += right_noise
    detections[:,6] += right_noise
    
    # false positives
    num_fp = rng.poisson(fp_rate)
    fp_dets = rng.uniform([0, -road_len, min_car_len],
                                [n_levels, road_len, max_car_len], size=(num_fp,3))
    fp_full_dets = np.zeros((num_fp, 7))
    fp_full_dets[:,0] = np.floor(fp_dets[:,0])
    fp_full_dets[:,1] = 1
    fp_full_dets[:,2] = fp_dets[:,1]
    fp_full_dets[:,3] = fp_dets[:,1]
    fp_full_dets[:,4] = 1
    fp_full_dets[:,5] = fp_dets[:,1]+fp_dets[:,2]
    fp_full_dets[:,6] = fp_dets[:,1]+fp_dets[:,2]
    detections = np.append(detections, fp_full_dets, axis=0)
    return detections

def sense_MWO(objects, rng=np.random):
    # false positives
    num_fp = rng.poisson(fp_rate)
    fp_dets = rng.uniform([0, -road_len, min_car_len, -1],
                                [n_levels, road_len, max_car_len, 1], size=(num_fp,4))
    fp_dets[:,0] = np.floor(fp_dets[:,0])
    objects = np.append(objects, fp_dets, axis=0)
    
    # false negatives
    keep_detections = rng.rand(objects.shape[0]) < detect_prob
    objects = objects[keep_detections]
    
    # noise
    measurements = np.array([getLR(pos, length, speed) for
                             obj_level, pos, length, speed in objects])
    measurements += rng.normal(scale=pos_noise_std, size=measurements.shape)
    measurements = np.append(objects[:,:1], measurements, axis=1)
#    pos_noise = rng.normal(scale=pos_noise_std, size=objects.shape[0])
#    objects[:,1] += pos_noise
#    length_noise = rng.normal(scale=pos_noise_std, size=objects.shape[0])
#    objects[:,2] += length_noise
    
    detections = []
    # determine which objects (and which parts of objects) are visible
    edges = [] # flip the openness of the space
    for level in range(n_levels):
        cross_len = crossing_len * (level+1)
        #for obj_level, pos, length, speed in objects:
        #    if obj_level != level: continue
        #    left_pos, right_pos = getLR(pos, length, speed)
        for left_pos, right_pos in measurements[measurements[:,0]==level,1:]:
            left_angle = left_pos/cross_len
            right_angle = right_pos/cross_len
            left_edge, right_edge = np.searchsorted(edges, [left_angle, right_angle])
            new_edges = []
            edge_visible_left = left_edge%2==0 # open space
            if left_edge == right_edge and not edge_visible_left:
                continue # full occlusion
            if edge_visible_left:
                edge_output_left = [True, left_pos, left_pos]
                new_edges += [left_angle]
            else:
                edge_output_left = [False, edges[left_edge-1]*cross_len,
                                           edges[left_edge]*cross_len]
            edge_visible_right = right_edge%2==0
            if edge_visible_right:
                edge_output_right = [True, right_pos, right_pos]
                new_edges += [right_angle]
            else:
                edge_output_right = [False, edges[right_edge-1]*cross_len,
                                           edges[right_edge]*cross_len]
            detections += [[level]+edge_output_left+edge_output_right]
            edges = edges[:left_edge] + new_edges + edges[right_edge:]
    detections = np.array(detections, dtype=float)
    return detections

def simpleInit(rng = np.random):
    planned_objects = []
    for level in range(n_levels_left):
        planned_objects += [(0, [level, -road_len, 4, 3.5])]
    for level in range(n_levels_right):
        planned_objects += [(0, [level+n_levels_left, road_len, 4, -3.5])]
    present_objects = []
    for step in range(0):
        present_objects, planned_objects, n_new_objects = update(
                                            present_objects, planned_objects, rng)
    return present_objects, planned_objects


    
# test and visualize simulator
nsteps = 100
burn_in = 0
ppm = 10
n_vert_pixels = int(crossing_len*(n_levels+1)*ppm)
n_horz_pixels = int(road_len*2*ppm)
im = np.zeros((n_vert_pixels, n_horz_pixels, 3), dtype=np.uint8) + 255
# little green square for your car/frog
im[-ppm:, n_horz_pixels/2-ppm:n_horz_pixels/2+ppm, :] = [10,255,10]
# useful info
n_objects = 0
speeds = []

if __name__ == '__main__':
    video_out = vwriter('frogger_sim.mkv',
                        inputdict={'-r':'5'}, outputdict={'-an':'-y'})
    
    # start with one planned object per level
    planned_objects = []
    for level in range(n_levels_left):
        planned_objects += [(0, [level, -road_len, 4, 3.5])]
    for level in range(n_levels_right):
        planned_objects += [(0, [level+n_levels_left, road_len, 4, -3.5])]
    present_objects = []
    # let it run a little while to randomize
    for step in range(burn_in):
        present_objects, planned_objects, n_new_objects = update(
                                            present_objects, planned_objects)
        
    for step in range(nsteps):
        present_objects, planned_objects, n_new_objects = update(
                                            present_objects, planned_objects)
        measurements = sense_OWO(present_objects)
        # log info
        n_objects += n_new_objects
        if n_new_objects > 0:
            speeds += [obj[3] for obj in present_objects[-n_new_objects:]]
            
        # plot real objects
        im[:-ppm,:,:] = 255
        for level, pos, length, speed in present_objects:
            left, right = getLR(pos, length, speed)
            left = max(0, int(left*ppm + n_horz_pixels/2))
            right = min(n_horz_pixels, int(right*ppm + n_horz_pixels/2))
            if left >= n_horz_pixels or right < 0 or right<left: continue
            top = int((n_levels - level - .1)*crossing_len*ppm)
            bottom = int((n_levels - level + .1)*crossing_len*ppm)
            drawBox2D(top,left,bottom,right, im, color=[0,0,0])
            
        # plot measurements
        for level, vis_left, left, left2, vis_right, right, right2 in measurements:
            left = max(0, int(left*ppm + n_horz_pixels/2))
            left2 = max(0, int(left2*ppm + n_horz_pixels/2))
            right = min(n_horz_pixels, int(right*ppm + n_horz_pixels/2))
            right2 = min(n_horz_pixels, int(right2*ppm + n_horz_pixels/2))
            if vis_right: assert right == right2
            if left >= n_horz_pixels or right2 < 0 or right2<left: continue
            top = int((n_levels - level - .2)*crossing_len*ppm)
            bottom = int((n_levels - level + .2)*crossing_len*ppm)
            if vis_left > 0 and vis_right > 0:
                drawBox2D(top, left, bottom, right, im, color=[10,10,255], linewidth=1)
            elif vis_left > 0:
                drawBox2D(top, left, bottom, right2, im, color=[10,10,255],linewidth=1,
                          sides_to_include=[False,True,True,False])
            elif vis_right > 0:
                drawBox2D(top, left, bottom, right2, im, color=[10,10,255],linewidth=1,
                          sides_to_include=[False,False,True,True])
            else:
                drawBox2D(top, left, bottom, right2, im, color=[10,10,255],linewidth=1,
                          sides_to_include=[False,False,True,False])
        video_out.writeFrame(im)
        
        if isCollision(present_objects):
            break
        
    video_out.close()
