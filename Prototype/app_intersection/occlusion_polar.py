# -*- coding: utf-8 -*-
"""
Created on Sat Oct 27 13:59:08 2018

@author: m2
"""
from bisect import bisect
from math import atan2, hypot, cos, sin, pi
import numba as nb

inf = 1e10
visual_resolution = .005 # radians, lines or gaps smaller than this are ignored
twopi = pi*2

@nb.jit(nb.b1(nb.f8,nb.f8,nb.f8,nb.f8,nb.f8,nb.f8), nopython=True)
def ccw(Ax,Ay, Bx,By, Cx,Cy):
    """Tests whether the turn formed by A, B, and C is ccw"""
    return (Bx - Ax) * (Cy - Ay) > (By - Ay) * (Cx - Ax)

@nb.jit(nb.b1(nb.f8,nb.f8,nb.f8,nb.f8), nopython=True)
def ccw0(Bx,By, Cx,Cy):
    """Whether the turn formed by (0,0), B, and C is ccw"""
    return Bx*Cy > By*Cx
    
@nb.jit(nb.b1(nb.f8,nb.f8,nb.f8,nb.f8,nb.f8,nb.f8,nb.f8,nb.f8), nopython=True)
def doLinesIntersect(A1x,A1y, A2x,A2y, B1x,B1y, B2x,B2y):
    return ccw(A1x,A1y, A2x,A2y, B1x,B1y) != ccw(A1x,A1y, A2x,A2y, B2x,B2y) and\
           ccw(B1x,B1y, B2x,B2y, A1x,A1y) != ccw(B1x,B1y, B2x,B2y, A2x,A2y)

def lineIntersection(A1, A2, B1, B2):
    dA = (A2[0]-A1[0], A2[1]-A1[1])
    dB = (B2[0]-B1[0], B2[1]-B1[1])
    crossA = A1[1]*A2[0]-A1[0]*A2[1]
    crossB = B1[1]*B2[0]-B1[0]*B2[1]
    denom = dB[1]*dA[0]-dB[0]*dA[1]
    # assume already checked for intersection, so denom != 0
    x = (crossA*dB[0] - crossB*dA[0])/denom
    y = (crossA*dB[1] - crossB*dA[1])/denom
    return x,y

@nb.jit(nb.f8(nb.f8,nb.f8,nb.f8,nb.f8,nb.f8,nb.f8), nopython=True)
def distOnLineAtSlope(x1,y1,x2,y2, xc,yc):
    num = x1*y2 - y1*x2
    denom = xc*(y2-y1) + yc*(x1-x2)
    if denom == 0: # all the points are in a line
        return y1/yc if xc == 0 else x1/xc
    return num / denom


def addToMap(occlusion_map, A1, A2):
    new_points = []
    # ensure that new segment is counterclockwise
    if ccw0(A2[0],A2[1], A1[0],A1[1]):
        A2, A1 = A1, A2
    # polar representation
    A1_angle = atan2(A1[1],A1[0])
    A1_dist = hypot(A1[1],A1[0])
    A2_angle = atan2(A2[1],A2[0])
    A2_dist = hypot(A2[1],A2[0])
    # if new segment is too slim visually, don't include it
    if (A2_angle-A1_angle)%twopi < visual_resolution:
        return occlusion_map, True
    
    # find segment to left of new segment (logn search)
    cl_seg_idx = bisect(occlusion_map, (A1_angle,)) - 1
    if cl_seg_idx == -1:
        cl_seg_idx = len(occlusion_map) - 1
    if cl_seg_idx == len(occlusion_map) - 1:
        cc_seg_idx = 0
    else:
        cc_seg_idx = cl_seg_idx + 1
    cl_segment = occlusion_map[cl_seg_idx]
    cc_segment = occlusion_map[cc_seg_idx]
    ## first check "real" segment
    # special case: new segment entirely in arc of this segment
    cl_angle, cl_prevdist, cl_dist = cl_segment
    cc_angle, cc_dist, cc_nextdist = cc_segment
    if cc_angle > cl_angle:
        assert A1_angle >= cl_angle and A1_angle <= cc_angle
        finished = A2_angle > cl_angle and A2_angle < cc_angle
    else:
        if (A1_angle < cl_angle) != (A1_angle < cc_angle):
            return occlusion_map, False
        if (A2_angle < cl_angle) == (A2_angle < cc_angle):
            if A2_angle > cl_angle and A1_angle < cl_angle:
                finished = False
            elif A2_angle < cl_angle and A1_angle > cl_angle:
                finished = True
            else:
                finished = A2_angle > A1_angle
        else:
            finished = False
        
    dist_old_on_new = distOnLineAtSlope(A1[0],A1[1],A2[0],A2[1],
                                        cos(cc_angle), sin(cc_angle))
    if cl_dist == inf:
        dist_new_on_old = inf
    else:
        B1 = (cos(cl_angle)*cl_dist, sin(cl_angle)*cl_dist)
        B2 = (cos(cc_angle)*cc_dist, sin(cc_angle)*cc_dist)
        dist_new_on_old = distOnLineAtSlope(B1[0],B1[1],B2[0],B2[1],
                                            A1[0],A1[1]) * A1_dist
    if dist_old_on_new < cc_dist and dist_new_on_old > A1_dist:
        new_points.append((A1_angle, dist_new_on_old, A1_dist))
        if finished:
            visible = True
        elif dist_old_on_new < cc_nextdist:
            visible = True
        else:
            visible = False
            new_points.append((cc_angle, dist_old_on_new, cc_nextdist))
    else:
        if dist_old_on_new < cc_dist - 1e-2 or dist_new_on_old > A1_dist + 1e-2:
            return occlusion_map, False
        if finished:
            visible = False
        elif dist_old_on_new < cc_nextdist:
            visible = True
            new_points.append((cc_angle, cc_dist, dist_old_on_new))
        else:
            visible = False
    
    total_loop = False
    
    cnt = 0        
    
    # now continue doing each segment until you reach end of new segment
    while not finished:
        cnt += 1
        assert cnt < 1000
        if cc_seg_idx == cl_seg_idx:
            total_loop = True
        cl_segment = cc_segment
        if cc_seg_idx == len(occlusion_map) - 1:
            cc_seg_idx = 0
        else:
            cc_seg_idx += 1
        cc_segment = occlusion_map[cc_seg_idx]
        cl_angle, cl_prevdist, cl_dist = cl_segment
        cc_angle, cc_dist, cc_nextdist = cc_segment
        if (cc_angle - cl_angle) % twopi > pi:
            # not actually a line, just a full semicircle or more of gap
            assert cl_segment[2] == inf
            break
        if total_loop: break
        if ccw0(A2[0],A2[1], cos(cc_angle), sin(cc_angle)):
            break
        dist_old_on_new = distOnLineAtSlope(A1[0],A1[1],A2[0],A2[1],
                    cos(cc_angle), sin(cc_angle))
        if cl_dist == inf:
            intersecting = False
        else:
            B1x = cos(cl_angle)*cl_dist
            B1y = sin(cl_angle)*cl_dist
            B2x = cos(cc_angle)*cc_dist
            B2y = sin(cc_angle)*cc_dist
            intersecting = doLinesIntersect(A1[0],A1[1], A2[0],A2[1], B1x,B1y, B2x,B2y)
        if intersecting: # will get there later
            return occlusion_map, False
        if dist_old_on_new < cc_dist:
            # new segment is still visible
            if not visible:
                return occlusion_map, False
            if dist_old_on_new > cc_nextdist:
                visible = False
                new_points.append((cc_angle, dist_old_on_new, cc_segment[2]))
        else:
            # new segment is still not visible
            if visible:
                return occlusion_map, False
            if dist_old_on_new < cc_nextdist:
                visible = True
                new_points.append((cc_angle, cc_dist, dist_old_on_new))
            else:
                new_points.append(cc_segment) # keep old point
    # handle end of new segment
    if cl_dist == inf:
        dist_new_on_old = inf
    else:
        B1 = (cos(cl_angle)*cl_dist, sin(cl_angle)*cl_dist)
        B2 = (cos(cc_angle)*cc_dist, sin(cc_angle)*cc_dist)
        dist_new_on_old = distOnLineAtSlope(B1[0],B1[1],B2[0],B2[1],
                                            A2[0],A2[1]) * A2_dist
    if dist_new_on_old > A2_dist:
        if not visible:
            return occlusion_map, False
        new_points.append((A2_angle, A2_dist, dist_new_on_old))
    else:
        if visible:
            return occlusion_map, False
    # now make new map
    if total_loop: # looked at entirety of old map
        # need to find where the new points wrapped around
        new_points_wrap = 0
        for k in xrange(1, len(new_points)):
            angle_1 = new_points[k-1][0]
            angle_2 = new_points[k][0]
            if angle_2 < angle_1:
                assert new_points_wrap is None
                new_points_wrap = k
        occlusion_map = new_points[new_points_wrap:] + new_points[:new_points_wrap]
    elif cl_seg_idx < cc_seg_idx: # not wrapping around end of loop
        occlusion_map = occlusion_map[:cl_seg_idx+1] +\
                        new_points + occlusion_map[cc_seg_idx:]
    else: # wraps around end
        # need to find where the new points wrapped around
        new_points_wrap = None
        for k in xrange(1, len(new_points)):
            angle_1 = new_points[k-1][0]
            angle_2 = new_points[k][0]
            if angle_2 < angle_1:
                assert new_points_wrap is None
                new_points_wrap = k
        if new_points_wrap is None:
            if new_points[0][0] < occlusion_map[cc_seg_idx][0]:
                new_points_wrap = 0
            else:
                assert new_points[0][0] >= occlusion_map[cl_seg_idx][0]
                new_points_wrap = len(new_points)
        occlusion_map = new_points[new_points_wrap:] +\
                        occlusion_map[cc_seg_idx:cl_seg_idx+1] +\
                        new_points[:new_points_wrap]
    return occlusion_map, True
         
def newOcclusionMap(starting_segment):
    starting_cl_angle = atan2(starting_segment[0][1], starting_segment[0][0])
    starting_cl_dist = hypot(starting_segment[0][1], starting_segment[0][0])
    starting_cc_angle = atan2(starting_segment[1][1], starting_segment[1][0])
    starting_cc_dist = hypot(starting_segment[1][1], starting_segment[1][0])
    occlusion_map = [(starting_cl_angle, inf, starting_cl_dist),
                     (starting_cc_angle, starting_cc_dist, inf)]
    if starting_cl_angle > starting_cc_angle:
        occlusion_map = occlusion_map[::-1]
    return occlusion_map
   

def makeMap(segments):
    starting_segment = segments[0]
    starting_cl_angle = atan2(starting_segment[0][1], starting_segment[0][0])
    starting_cl_dist = hypot(starting_segment[0][1], starting_segment[0][0])
    starting_cc_angle = atan2(starting_segment[1][1], starting_segment[1][0])
    starting_cc_dist = hypot(starting_segment[1][1], starting_segment[1][0])
    occlusion_map = [(starting_cl_angle, inf, starting_cl_dist),
                     (starting_cc_angle, starting_cc_dist, inf)]
    if starting_cl_angle > starting_cc_angle:
        occlusion_map = occlusion_map[::-1]
    for segment in segments[1:]:
        A1 = segment[0]
        A2 = segment[1]
        occlusion_map = addToMap(occlusion_map, A1, A2)
    # to make it a ring
    breakpoint = bisect(occlusion_map, (0.,))
    occlusion_map += [(angle-pi, d1, d2) for angle, d1, d2 in occlusion_map[breakpoint:]]
    occlusion_map += [(angle+pi, d1, d2) for angle, d1, d2 in occlusion_map[:breakpoint]]
        
    
def pruneMap(occlusion_map):
    # remove segments that are shorter than visual_resolution
    new_map = []
    point = occlusion_map[0]
    for nextpoint in occlusion_map[1:]:
        if nextpoint[0] - point[0] > visual_resolution:
            new_map.append(point)
            point = nextpoint
        else:
            point = (point[0], point[1], nextpoint[2])
    new_map.append(point)
    # now make a second version going from 0 to 2pi instead of -pi to pi
    cut = bisect(new_map, (0.,))
    new_map += [(angle-pi,d1,d2) for angle,d1,d2 in new_map[cut:]]
    new_map += [(angle+pi,d1,d2) for angle,d1,d2 in new_map[:cut]]
    return new_map
    
def doubleUp(occlusion_map):
    cut = bisect(occlusion_map, (0.,))
    occlusion_map += [(angle-pi,d1,d2) for angle,d1,d2 in occlusion_map[cut:]]
    occlusion_map += [(angle+pi,d1,d2) for angle,d1,d2 in occlusion_map[:cut]]
    return occlusion_map


# currently no factoring intersections
@nb.jit(nb.f8(nb.f8[:],nb.f8[:],nb.f8[:,:]), nopython=True)
def occludedByMap(A1, A2, occlusion_map):
    n = len(occlusion_map) // 2
    A1_angle = atan2(A1[1], A1[0])
    A2_angle = atan2(A2[1], A2[0])
    midpoint_on_right = A1[0]+A2[0] >= 0
    if midpoint_on_right:
        lo = 0
        hi = n
    else:
        lo = n
        hi = 2*n
        A1_angle -= pi
        A2_angle -= pi
        if A2_angle < A1_angle: A2_angle += twopi
    visible_radians = 0.
    previous_angle = A1_angle
    # bisection - from https://github.com/python/cpython/blob/2.7/Lib/bisect.py
    while lo < hi:
        mid = (lo+hi)//2
        if A1_angle < occlusion_map[mid][0]: hi = mid
        else: lo = mid+1
    seg_idx = lo
    current_angle, current_distance, next_distance = occlusion_map[seg_idx]
    num = A1[0]*A2[1] - A2[0]*A1[1] # substeps of distOnLineAtSlope function
    dA = (A2[1]-A1[1], A1[0]-A2[0])
    while current_angle > previous_angle and current_angle < A2_angle:
        denom = cos(current_angle)*dA[0] + sin(current_angle)*dA[1]
        if denom == 0:
            if cos(current_angle) == 0:
                distonAatseg = A1[1] / sin(current_angle)
            else:
                distonAatseg = A1[0] / cos(current_angle)
        else:
            distonAatseg = num/denom
        if distonAatseg < current_distance:
            visible_radians += current_angle - previous_angle
        previous_angle = current_angle
        seg_idx += 1
        if seg_idx == n or seg_idx == 2*n:
            break # should do this cleaner, but meh
        current_angle, current_distance, next_distance = occlusion_map[seg_idx]
    # see if end is visible
    denom = cos(previous_angle)*dA[0] + sin(previous_angle)*dA[1]
    if denom == 0:
        if cos(previous_angle) == 0:
            distonAatseg = A1[1] / sin(previous_angle)
        else:
            distonAatseg = A1[0] / cos(previous_angle)
    else:
        distonAatseg = num/denom
    if distonAatseg < current_distance:
        visible_radians += A2_angle - previous_angle
    return visible_radians



if __name__ == '__main__':
    segments = [((2.,-2.),(2.,-1.)),((3.,1.),(3.,2.))]
    #segments = [((4.,-2.),(4.,2.)), ((3.,-.5),(3.,.5))]
    #segments = [((3.,-2.),(3.,2.)), ((4.,-1.),(4.,-1.))]
    starting_segment = segments[0]
    occlusion_map = newOcclusionMap(starting_segment)
    for segment in segments[1:]:
        A1 = segment[0]
        A2 = segment[1]
        new_occlusion_map = addToMap(occlusion_map, A1, A2)
    print new_occlusion_map
    # to make it a ring
    #occlusion_map = occlusion_map + 1