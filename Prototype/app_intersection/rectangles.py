#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 7/31/18 occlude() function just for boxTracker, recursive
7/30/18 flip clockwise occluding objects
7/22/18 minor fix to distOnLineAtSlope, and faster getVisibleSegments

rectangle = x,y of center , angle , l,w (half of total length and width)

based off rectools from pseudosensor
and collisionCheck from VCD
"""

import numpy as np
import scipy.spatial as spspatial
from scipy.optimize import linprog
#https://www.toptal.com/python/computational-geometry-in-python-from-theory-to-implementation
#def ccw(A, B, C):
#    """Tests whether the turn formed by A, B, and C is ccw"""
#    return (B.x - A.x) * (C.y - A.y) > (B.y - A.y) * (C.x - A.x)

def collisionCheck(veh1, veh2):
    off_x = veh2[...,0] - veh1[...,0]
    off_y = veh2[...,1] - veh1[...,1]
    c1 = np.cos(veh1[...,2])
    s1 = np.sin(veh1[...,2])
    c2 = np.cos(veh2[...,2])
    s2 = np.sin(veh2[...,2])
#    # because given point is front and center
#    off_x += c1*vlen - c2*vlen
#    off_y += s1*vlen - s2*vlen
    
    off_c = abs(c1*c2+s1*s2)
    off_s = abs(c1*s2-s1*c2)
    
    return (abs(c1*off_x + s1*off_y) <
             veh1[...,3] + off_c*veh2[...,3] + off_s*veh2[...,4]) &\
           (abs(s1*off_x - c1*off_y) <
             veh1[...,4] + off_s*veh2[...,3] + off_c*veh2[...,4]) &\
           (abs(c2*off_x + s2*off_y) <
             veh2[...,3] + off_c*veh1[...,3] + off_s*veh1[...,4]) &\
           (abs(s2*off_x - c2*off_y) <
             veh2[...,4] + off_s*veh1[...,3] + off_c*veh1[...,4])

def ccwOrig(a, b):
    return a[:,0]*b[:,1] > b[:,0]*a[:,1]

# standard format - x,y,angle,l,w
# docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.HalfspaceIntersection.html
def overlap(rec1, rec2):
    if not collisionCheck(rec1, rec2): return 0.
            
    x1,y1,theta1,l1,w1 = rec1
    x2,y2,theta2,l2,w2 = rec2
    cos1 = np.cos(theta1)
    sin1 = np.sin(theta1)
    diffx = x2-x1; diffy = y2-y1
    x2 = diffx*cos1 + diffy*sin1
    y2 = -diffx*sin1 + diffy*cos1
    cos2 = np.cos(theta2-theta1)
    sin2 = np.sin(theta2-theta1)
    
    A = np.array([[1,0,1],[-1,0,1],[0,1,1],[0,-1,1],
                  [cos2,sin2,1],[-cos2,-sin2,1],[-sin2,cos2,1],[sin2,-cos2,1]])
    b = np.array([l1,l1,w1,w1, l2+cos2*x2+sin2*y2, l2-cos2*x2-sin2*y2,
                  w2-sin2*x2+cos2*y2, w2+sin2*x2-cos2*y2])
    res = linprog(np.array([0,0,-1]), A_ub=A, b_ub=b, bounds=(-np.inf,np.inf))
    assert res.success
    interior = res.x[:2]
    
    halfspaces = np.append(A[:,:2], -b[:,None], axis=1)
    hs = spspatial.HalfspaceIntersection(halfspaces, interior).intersections
    return spspatial.ConvexHull(hs).volume
    
def IoU(rec1, rec2):
    itx = overlap(rec1, rec2)
    return itx / (rec1[3]*rec1[4]*4 + rec2[3]*rec2[4]*4 - itx)


#def visibleForm(x,y,l,w,cos,sin):
#    """ find origin realigned to vehicle axes, scaled by length and width
#        a useful form for assessing visibility from the origin
#        input = [x,y,angle,length,width] (array or matrix)
#        output = [x, y] (array or matrix)"""
#    return np.array(((-cos*x - sin*y) / l , (-cos*y + sin*x) / w))
#
#def getVisibleSegments(rect):
#    """ returns segments that are visible from the origin
#        segment = [x1, y1, x2, y2], going ccw from origin's pov
#        only works for one rectangle """
#    x,y,angle,l,w = rect
#    cos = np.cos(angle)
#    sin = np.sin(angle)
#    point = visibleForm(x,y,l,w,cos,sin)
#    if np.abs(point[0]) <= 1:
#        if point[1] > 0:
#            out = [[-1,1,1,1]]
#        else:
#            out = [[1,-1,-1,-1]]
#    elif np.abs(point[1]) <= 1:
#        if point[0] > 0:
#            out = [[1,1,1,-1]]
#        else:
#            out = [[-1,-1,-1,1]]
#    elif point[0] > 0:
#        if point[1] > 0:
#            out = [[-1,1,1,1],[1,1,1,-1]]
#        else:
#            out = [[1,1,1,-1],[1,-1,-1,-1]]
#    else:
#        if point[1] > 0:
#            out = [[-1,-1,-1,1],[-1,1,1,1]]
#        else:
#            out = [[1,-1,-1,-1],[-1,-1,-1,1]]
#    out = np.array(out, dtype=float)
#    x1,y1,x2,y2 = out.T
#    x1 *= rect[3]
#    x2 *= rect[3]
#    y1 *= rect[4]
#    y2 *= rect[4]
#    out = out.copy()
#    out[:,0] = x1 * cos - y1 * sin + rect[0]
#    out[:,2] = x2 * cos - y2 * sin + rect[0]
#    out[:,1] = y1 * cos + x1 * sin + rect[1]
#    out[:,3] = y2 * cos + x2 * sin + rect[1]
#    return out#np.append(out[:1,:2], out[-1:,2:4], axis=0)

def getVisibleSegments(rect):
    x,y,angle,l,w = rect
    cos = np.cos(angle)
    sin = np.sin(angle)
    p0 = cos*x + sin*y
    p1 = sin*x - cos*y
    if abs(p0) <= l:
        if p1 > 0:
            out = [[-1,1,1,1]]
        else:
            out = [[1,-1,-1,-1]]
    elif abs(p1) <= w:
        if p0 < 0:
            out = [[1,1,1,-1]]
        else:
            out = [[-1,-1,-1,1]]
    elif p0 < 0:
        if p1 > 0:
            out = [[-1,1,1,1],[1,1,1,-1]]
        else:
            out = [[1,1,1,-1],[1,-1,-1,-1]]
    else:
        if p1 > 0:
            out = [[-1,-1,-1,1],[-1,1,1,1]]
        else:
            out = [[1,-1,-1,-1],[-1,-1,-1,1]]
    out = np.array(out, dtype=float)
    x1,y1,x2,y2 = out.T
    x1 *= l
    x2 *= l
    y1 *= w
    y2 *= w
    out = out.copy()
    out[:,0] = x1 * cos - y1 * sin + x
    out[:,2] = x2 * cos - y2 * sin + x
    out[:,1] = y1 * cos + x1 * sin + y
    out[:,3] = y2 * cos + x2 * sin + y
    return out
    

lidar_angle_res = .007
def distOnLineAtSlope(x1,y1,x2,y2, xc,yc):
    num = x1*y2 - y1*x2
    denom = xc*(y2-y1) + yc*(x1-x2)
    if denom == 0: # all the points are in a line
        return y1/yc if xc == 0 else x1/xc
    return num / denom
def ccw(x1,y1,x2,y2): return x1*y2 > y1*x2 + lidar_angle_res
def findCoverage(rect, objects):
    seg_a = getVisibleSegments(rect)
    ax1, ay1 = seg_a[0,:2]
    ax2, ay2 = seg_a[-1,2:4]
    unchanged_1 = True
    unchanged_2 = True
    for obj_b in objects:
        if obj_b.shape[0]==1:
            bx1, by1, bx2, by2 = obj_b[0]
        elif obj_b.shape[1] == 4: # assume obtained by getVisibleSegments
            bx1, by1 = obj_b[0,:2]
            bx2, by2 = obj_b[-1,2:4]
        elif obj_b.shape[1] == 3: # assume measurements returned by findCoverage
            bx1, by1 = obj_b[0,:2]
            bx2, by2 = obj_b[-1,:2]
        else:
            raise Exception
        if not ccw(bx1, by1, bx2, by2): # some objects might be in wrong order
            bx1, by1, bx2, by2 = bx2, by2, bx1, by1
        if not (ccw(bx2,by2,ax1,ay1) or ccw(ax2,ay2,bx1,by1)):
            right = ccw(ax1,ay1,bx1,by1)
            left = ccw(bx2,by2,ax2,ay2)
            if left and right: # seg a spans seg b
                # for now, we assume that system can figure this out
                # in future: split into two objects
                pass
            elif right:
                adnew = distOnLineAtSlope(ax1,ay1,ax2,ay2,bx1,by1)
                if adnew > 1: # seg b in front of seg a
                    ax2 = adnew*bx1
                    ay2 = adnew*by1
                    unchanged_2 = False
            elif left:
                adnew = distOnLineAtSlope(ax1,ay1,ax2,ay2,bx2,by2)
                if adnew > 1:
                    ax1 = adnew*bx2
                    ay1 = adnew*by2
                    unchanged_1 = False
            else: # seg b spans a
                bd = distOnLineAtSlope(bx1,by1,bx2,by2,ax1,ay1)
                if bd < 1: # seg b in front of seg a
                    return np.zeros((0,3)), 0
    visible_area = (ax1*ay2 - ay1*ax2) / np.hypot(ax1,ay1) / np.hypot(ax2,ay2)
    if visible_area < lidar_angle_res:
        return np.zeros((0,3)), 0
    elif seg_a.shape[0] == 1: # just this segment
        pts = [[ax1, ay1, unchanged_1], [ax2,ay2, unchanged_2]]
    else: # include corner (maybe)
        x1, y1, xcorn, ycorn = seg_a[0,:]
        x2, y2 = seg_a[1,2:4]
        visible_left_of_corner = ccw(ax1,ay1,xcorn,ycorn)
        visible_right_of_corner = ccw(xcorn, ycorn, ax2,ay2)
        if not unchanged_1:
            if visible_left_of_corner:
                correct_1 = distOnLineAtSlope(x1, y1, xcorn, ycorn, ax1, ay1)
            else:
                correct_1 = distOnLineAtSlope(x2, y2, xcorn, ycorn, ax1, ay1)
            ax1 *= correct_1
            ay1 *= correct_1
        if not unchanged_2:
            if visible_right_of_corner:
                correct_2 = distOnLineAtSlope(x2, y2, xcorn, ycorn, ax2, ay2)
            else:
                correct_2 = distOnLineAtSlope(x1, y1, xcorn, ycorn, ax2, ay2)
            ax2 *= correct_2
            ay2 *= correct_2
        if visible_left_of_corner and visible_right_of_corner:
            pts = [[ax1,ay1,unchanged_1], [xcorn, ycorn, True], [ax2,ay2,unchanged_2]]
        else:
            pts = [[ax1,ay1,unchanged_1], [ax2,ay2,unchanged_2]]
    return np.array(pts), visible_area

### following two functions are specifically for use with boxTracker
def occlude(rect, objects):
    seg_a = getVisibleSegments(rect)
    ax1, ay1 = seg_a[0,:2]
    ax2, ay2 = seg_a[-1,2:4]
    return visibility(ax1, ay1, ax2, ay2, objects)

def visibility(ax1, ay1, ax2, ay2, objects):
    for b, obj_b in enumerate(objects):
        # assume measurements returned by findCoverage
        bx1, by1, bx2, by2 = obj_b
        if not ccw(bx1, by1, bx2, by2): # some objects might be in wrong order
            bx1, by1, bx2, by2 = bx2, by2, bx1, by1
        if not (ccw(bx2,by2,ax1,ay1) or ccw(ax2,ay2,bx1,by1)):
            right = ccw(ax1,ay1,bx1,by1)
            left = ccw(bx2,by2,ax2,ay2)
            if left and right: # seg a spans seg b
                adnew1 = distOnLineAtSlope(ax1,ay1,ax2,ay2,bx1,by1)
                adnew2 = distOnLineAtSlope(ax1,ay1,ax2,ay2,bx2,by2)
                # for non-intersecting lines, both should be higher or lower
                # but we are checking hypothetical objects...
                if min(adnew1, adnew2) > 1: # all higher
                    visible_1 = visibility(ax1, ay1, adnew1*bx1, adnew1*by1,
                                           objects[b:])
                    visible_2 = visibility(adnew2*bx2,adnew2*by2, ax2, ay2,
                                           objects[b:])
                    return max(visible_1, visible_2)
            elif right:
                adnew = distOnLineAtSlope(ax1,ay1,ax2,ay2,bx1,by1)
                if adnew > 1.01: # seg b in front of seg a
                    ax2 = adnew*bx1
                    ay2 = adnew*by1
            elif left:
                adnew = distOnLineAtSlope(ax1,ay1,ax2,ay2,bx2,by2)
                if adnew > 1.01:
                    ax1 = adnew*bx2
                    ay1 = adnew*by2
            else: # seg b spans a
                bd = distOnLineAtSlope(bx1,by1,bx2,by2,ax1,ay1)
                if bd < 1: # seg b in front of seg a
                    return 0
    visible_area = (ax1*ay2 - ay1*ax2) / np.hypot(ax1,ay1) / np.hypot(ax2,ay2)
    return visible_area


"""
input: edge format [[x, y, open],...]
output: x,y,theta,l,w for median values of l,w , plus range of l and w
obviously this form is not unique even with both sides fully visible
"""
def edgesToRect(edges, max_l, max_w):
    vector1 = (edges[1,0]-edges[0,0], edges[1,1]-edges[0,1])
    min_l = np.hypot(vector1[0], vector1[1])/2
    vector1 = (vector1[0]/min_l/2, vector1[1]/min_l/2)
    if edges.shape[0] == 3:
        if edges[0,2]: max_l = min_l
        vector2 = (edges[1,0]-edges[2,0], edges[1,1]-edges[2,1])
        min_w = np.hypot(vector2[0], vector2[1])/2
        vector2 = (vector2[0]/min_w/2, vector2[1]/min_w/2)
        # debug: assert perpendicular edges
        assert abs(vector1[0]*vector2[0]+vector1[1]*vector2[1]) < 1e-2
        if edges[2][2]: max_w = min_w
    else:
        if edges[0][2] and edges[1,2]: max_l = min_l
        min_w = 0.01
        vector2 = (vector1[1], -vector1[0])
        # facing towards visible edge
        if vector2[0]*edges[0][0] + vector2[1]*edges[0][1] > 0:
            vector2 = (-vector2[0], -vector2[1])
    max_l = max(max_l, min_l)
    mean_l = min_l/2 + max_l/2
    max_w = max(max_w, min_w)
    mean_w = min_w/2 + max_w/2
    mean_x = edges[1,0] - vector1[0]*mean_l - vector2[0]*mean_w
    mean_y = edges[1,1] - vector1[1]*mean_l - vector2[1]*mean_w
    angle = np.arctan2(vector1[1], vector1[0])
    angle = np.mod(angle, np.pi)
    return mean_x, mean_y, angle, mean_l, mean_w, max_l-min_l, max_w-min_w