#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 6/15/18 made compatible with Python 3
lane = +1 for right side, -1 for left side, +2 for far right (side area),
-2 for far left, None for too far to be assigned to road
"""

import numpy as np
from prototype.app_highwaymerge.options import road_file

## get road shape from text file
with open(road_file, 'r') as roadfile: filestring = roadfile.read()
filelines = [line.strip() for line in filestring.split('\n')]
# get road points
assert filelines[0] == 'road'
points = []
curve = []
line_idx = 1
while filelines[line_idx] != '':
    fileline = filelines[line_idx].split(' ')
    points += [(float(fileline[0]), float(fileline[1]))]
    curve += [bool(int(fileline[2]))]
    line_idx += 1
assert len(points) > 1
points = np.array(points)
curve = curve[:-1]
# get lane distances
while filelines[line_idx] != 'lane distances':
    line_idx += 1
line_idx += 1
lane_dist_distinction = np.array([float(lanedist) for lanedist in
                                  filelines[line_idx].split(' ')])
assert len(lane_dist_distinction) == 5
lane_values = [None] + list(range(-(len(lane_dist_distinction)//2), 0)) +\
                       list(range(1, len(lane_dist_distinction)//2+1)) + [None]
lane_dist_assign = lane_dist_distinction[:-1]/2. + lane_dist_distinction[1:]/2.

## or hardcode road shape
# Balcones
#points = np.array([(621339.2685035883, 3363166.1239765743),
#                   (621315.6788662691, 3363213.522826928),
#                   (621317.8473219264, 3363260.322254533),
#                   (621365.8841396358, 3363355.6279183105),
#                   (621446.5971506495, 3363476.0164087024)])
#curve = [False, True, False, False]

# indoor practice stadium
# merge at 622415.591, 3351125.076
# middle = 622404.64, 3351098.35
#points = np.array([(622386.75, 3350839.78),
#                   (622342.84, 3350852.48),
#                   (622452.72, 3351287.98)])
#curve = [False, False]

# innovation and library center
# merge at 622335.195, 3361937.51
#points = np.array([(621862.5503612919, 3362206.3187342556),
#                   (622354.8664308921, 3361931.5239201584),
#                   (622481.5621648553, 3361885.508746708)])
#curve = [False, False]

#lane_dist_distinction = np.array([-10, -4, 0, 4, 10])
#lane_dist_assign = [-7, -2, 2, 7]
#lane_values = [None,-2,-1,1,2,None]


## process road shape
nroads = points.shape[0] - 1
along_vec = np.diff(points, axis=0)
lengths = np.hypot(along_vec[:,0], along_vec[:,1])
sumlengths = np.append([0], np.cumsum(lengths))
along_vec /= lengths[:,None]
across_vec = along_vec.dot([[0,-1],[1,0]])

curve_centers = np.zeros((nroads,2))
curve_mags = np.zeros((nroads,))
curve_start_angles = np.zeros((nroads,))
curve_end_angles = np.zeros((nroads,))
curve_directions = np.zeros((nroads,))
def ccw(a,b,c):
    return np.sign((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))
def fixRad(rad):
    return (rad + np.pi) % (2*np.pi) - np.pi
for k in range(nroads):
    if curve[k]:
        intersection_mtx = along_vec[[k-1,k+1],:]
        intersection_vec = np.einsum(intersection_mtx,[0,1],points[k:k+2,:],[0,1],[0])
        curve_centers[k] = np.linalg.solve(intersection_mtx, intersection_vec)
        vec1 = points[k] - curve_centers[k]
        vec2 = points[k+1] - curve_centers[k]
        curve_mag1 = np.hypot(*vec1)
        curve_mag2 = np.hypot(*vec2)
        #print curve_mag1 - curve_mag2
        curve_mags[k] = curve_mag1/2 + curve_mag2/2
        curve_angle1 = np.arctan2(*vec1[::-1])
        curve_angle2 = np.arctan2(*vec2[::-1])
        curve_start_angles[k] = curve_angle1
        curve_end_angles[k] = curve_angle2
        diff_angle = fixRad(curve_angle2 - curve_angle1)
        lengths[k] = np.abs(diff_angle)*curve_mags[k]
        curve_directions[k] = np.sign(diff_angle)

tol = 1 # m

def global2Road(xy):
    xy_off = xy - points[:-1]
    along_vals = np.einsum(along_vec, [0,1], xy_off, [0,1], [0])
    lateral_distance = np.abs(np.einsum(across_vec, [0,1], xy_off, [0,1], [0]))
    for k in range(nroads):
        if curve[k]:
            from_center = xy - curve_centers[k]
            mag = np.hypot(*from_center)
            angle = np.arctan2(*from_center[::-1])
            along_vals[k] = fixRad(angle - curve_start_angles[k])*\
                            curve_mags[k]*curve_directions[k]
            lateral_distance[k] = np.abs(mag - curve_mags[k])
			
    within_road = (along_vals > 0 - tol) & (along_vals < lengths + tol)
    if not np.any(within_road): return 0., None
    road = np.argmin(lateral_distance - 10000*within_road)
    lane = lane_values[np.searchsorted(lane_dist_distinction, lateral_distance[road])]
    #lane = 0 if lateral_distance[road] < 4 else None
    return np.sum(lengths[:road]) + along_vals[road], lane

def road2Global(pos, lane=0):
    road = np.searchsorted(sumlengths[1:], pos)
    if road >= nroads: # past road limit
        xy = points[-1]
    elif curve[road]:
        curve_center = curve_centers[road]
        curve_angle = curve_start_angles[road]
        curve_angle += (pos - sumlengths[road])*curve_directions[road]/curve_mags[road]
        xy = curve_center + [np.cos(curve_angle)*curve_mags[road],
                             np.sin(curve_angle)*curve_mags[road]]
    else:
        xy = points[road] + along_vec[road]*(pos - sumlengths[road])
    if lane != 0 and not (lane is None):
        lane_dist_idx = lane+2 if lane < 0 else lane+1
        xy += across_vec[road]*lane_dist_assign[lane_dist_idx]
    return xy

## for curve: v1^T pc = v1T p1; p2T pc = v2T p2
## within_road = ccw(pc,p1,p) & ccw(pc,p,p2) & ccw(pc,p1,p2) | (all cw)
