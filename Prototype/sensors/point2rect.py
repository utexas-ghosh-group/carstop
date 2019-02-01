# -*- coding: utf-8 -*-
"""
last mod 11/4/18
adding merged measurement thinking
"""

import numpy as np
from math import hypot, atan2 # can be faster than calling numpy

# copied from numpy's isclose - without all the corner cases that slow it down
def isclose(x, y, rtol=1e-5, atol=1e-8):
    return abs(x-y) <= atol + rtol * abs(y)

# linear regression with euclidean, rather than y, error
def linReg(sumxx, sumyy, sumxy):
    b = sumxx - sumyy
    mag = np.hypot(2*sumxy, b)
    if mag < 1e-10: # basically white noise, doesn't matter what your line is
        return 1., 0., sumxx
    vx2 = .5 + .5 * b / mag
    vy = np.sqrt(1 - vx2)
    vx = np.sqrt(vx2)
    if sumxy < 0:
        vy *= -1 # switch sign for either vx or vy
    sse = .5 * (sumxx + sumyy - mag) # sum squared error
    return vx, vy, sse

# line fit, inclusive with corners
def getRectangleFitError(points, corneridx):
    mean1 = np.mean(points[:corneridx+1], axis=0)
    points1 = points[:corneridx+1] - mean1
    mean2 = np.mean(points[corneridx:], axis=0)
    points2 = points[corneridx:] - mean2
    sumxx = np.sum(points1[:,0]*points1[:,0]) + np.sum(points2[:,1]*points2[:,1])
    sumyy = np.sum(points1[:,1]*points1[:,1]) + np.sum(points2[:,0]*points2[:,0])
    sumxy = np.sum(points1[:,0]*points1[:,1]) - np.sum(points2[:,0]*points2[:,1])
    vecx, vecy, error = linReg(sumxx, sumyy, sumxy)
    return error

# for every corner candidate point, finds deviations and calculates best fit error
def getBestFit(points):
    n = points.shape[0]
    nrange = np.arange(1,n+1, dtype=float)
    sum1 = np.cumsum(points, axis=0)
    sum2 = np.cumsum(points[::-1], axis=0)[::-1]
    xx = points[:,0]*points[:,0]
    xy = points[:,0]*points[:,1]
    yy = points[:,1]*points[:,1]
    sumxx = np.cumsum(xx) - np.square(sum1[:,0]) / nrange
    sumxx += np.cumsum(yy[::-1])[::-1] - np.square(sum2[:,1]) / nrange[::-1]
    sumxy = np.cumsum(xy) - sum1[:,0]*sum1[:,1] / nrange
    sumxy -= np.cumsum(xy[::-1])[::-1] - sum2[:,0]*sum2[:,1] / nrange[::-1]
    sumyy = np.cumsum(yy) - np.square(sum1[:,1]) / nrange
    sumyy += np.cumsum(xx[::-1])[::-1] - np.square(sum2[:,0]) / nrange[::-1]
    mag = np.hypot(2*sumxy, sumxx - sumyy)
    sse = .5 * (sumxx + sumyy - mag) # sum squared error
    best_idx = np.argmin(sse)
    if sse[best_idx] <= 0: # roundoff error, basically perfect fit
        return best_idx, 0.
    best_rmse = np.sqrt(sse[best_idx] / n)
    return best_idx, best_rmse

min_edge_len = .1
def getCorners(points, corneridx):
    edges = np.zeros((3,2))
    if corneridx == 0 or corneridx == points.shape[0]-1:
        # just a line
        mean1 = np.mean(points, axis=0)
        points1 = points - mean1
        sumxx = np.sum(points1[:,0]*points1[:,0])
        sumyy = np.sum(points1[:,1]*points1[:,1])
        sumxy = np.sum(points1[:,0]*points1[:,1])
        vecx, vecy, error = linReg(sumxx, sumyy, sumxy)
        projections = vecx*points1[:,0] + vecy*points1[:,1]
        if projections[-1] > projections[0]:
            projection1 = min(projections)
            projection2 = max(projections)
        else:
            projection1 = max(projections)
            projection2 = min(projections)
        edges[0] = mean1 + (vecx*projection1, vecy*projection1)
        edges[1] = mean1 + (vecx*projection2, vecy*projection2)
        return edges[:2]
    mean1 = np.mean(points[:corneridx+1], axis=0)
    points1 = points[:corneridx+1] - mean1
    mean2 = np.mean(points[corneridx:], axis=0)
    points2 = points[corneridx:] - mean2
    sumxx = np.sum(points1[:,0]*points1[:,0]) + np.sum(points2[:,1]*points2[:,1])
    sumyy = np.sum(points1[:,1]*points1[:,1]) + np.sum(points2[:,0]*points2[:,0])
    sumxy = np.sum(points1[:,0]*points1[:,1]) - np.sum(points2[:,0]*points2[:,1])
    vecx, vecy, error = linReg(sumxx, sumyy, sumxy)
    # find beginning of first line segment
    projections = vecx*points1[:,0] + vecy*points1[:,1]
    if projections[-1] > projections[0]:
        projection = min(projections)
    else:
        projection = max(projections)
    end1 = mean1 + (vecx*projection, vecy*projection)
    projections = vecy*points2[:,0] - vecx*points2[:,1]
    if projections[-1] > projections[0]:
        projection = max(projections)
    else:
        projection = min(projections)
    end2 = mean2 + (vecy*projection, -vecx*projection)
    edges[0] = end1
    edges[2] = end2
    # determining if it's worth adding corner
    projection1 = vecx*(end2[0]-end1[0]) + vecy*(end2[1]-end1[1])
    projection2 = vecy*(end1[0]-end2[0]) - vecx*(end1[1]-end2[1])
    if abs(projection1) < min_edge_len or abs(projection2) < min_edge_len:
        return edges[[0,2]]
    # add corner
    edges[1] = (end1[0] + projection1*vecx, end1[1] + projection1*vecy)
    assert isclose(edges[1,0], end2[0] + projection2*vecy) and\
           isclose(edges[1,1], end2[1] - projection2*vecx)
    return edges
    

# storing x,y ---> 5w+2 arithmetic ops and 2 trig ops
# storing r,theta ---> 8w arithmetic ops and w trig ops
# where w = expected number of distance checks per point
# but you would still convert to x,y eventually... so better to use x,y
series_distance_cutoff = .3 ** 2 # m #.5
outlier_skipped_distance_cutoff = .6 ** 2# m #.8
def addPoint(point, points):
    if len(points) < 2:
        points.append(point)
        return None

    lastpoint = points[-1]
    beforepoint = points[-2]
    dx = lastpoint[0]-beforepoint[0]
    dy = lastpoint[1]-beforepoint[1]
    if dx*dx + dy*dy > series_distance_cutoff:
        points.pop(-1)

        dx = point[0]-lastpoint[0]
        dy = point[1]-lastpoint[1]
        if dx*dx + dy*dy > series_distance_cutoff:

            dx = point[0]-beforepoint[0]
            dy = point[1]-beforepoint[1]
            if dx*dx + dy*dy > outlier_skipped_distance_cutoff:
                returnval = np.array(points) if len(points) > 2 else None # OR 1
                for k in xrange(len(points)): points.pop()
                points.append(point)
                return returnval
            else:
                points.append(point)
                return None
        else:
            returnval = np.array(points) if len(points) > 2 else None # OR 1
            for k in xrange(len(points)): points.pop()
            points.append(lastpoint)
            points.append(point)
            return returnval
    else:
        points.append(point)
        return None
    
    
# take two segments, assume points evenly distributed on them
# find best single line fit
merging_sin_distance_cutoff = .15
merged_line_rmse_cutoff = .5
min_edge_len_merging = .15 # if far edge is shorter than this, can ignore it
def mergeSegments(segA, segB):    
    A1x, A1y = segA[-2,:2]
    A2x, A2y = segA[-1,:2]
    B1x, B1y = segB[0,:2]
    B2x, B2y = segB[1,:2]
    jump_x = B1x-A2x
    jump_y = B1y-A2y
    jumpdistance = hypot(jump_x, jump_y)
    
    A_corner = False
    if segA.shape[0]==3:
        A_hidden_edge = hypot(A1x-segA[0,0], A1y-segA[0,1])
        if A_hidden_edge > min_edge_len_merging:
            A_corner = True
    B_corner = False
    if segB.shape[0]==3:
        B_hidden_edge = hypot(B2x-segB[2,0], B2y-segB[2,1])
        if B_hidden_edge > min_edge_len_merging:
            B_corner = True
    # can't see three sides - so if both segments have two sides, ditch it
    if A_corner and B_corner:
        #print("merged cancelled bc too many corners")
        return None
    
    # get line parameters for first segment
    Amx = (A1x+A2x)/2
    Amy = (A1y+A2y)/2
    Adx = A2x-A1x
    Ady = A2y-A1y
    Alen = hypot(Adx, Ady)
    # if segment is small enough, can maybe ignore its current orientation
    if Alen < jumpdistance and Alen < min_edge_len_merging:
        A1x, A1y = Amx, Amy
        A2x, A2y = Amx, Amy
        Adx = 0
        Ady = 0
        Alen = .1 # just to prevent 0-division
        
    # get line parameters for second segment
    Bmx = (B1x+B2x)/2
    Bmy = (B1y+B2y)/2
    Bdx = B2x-B1x
    Bdy = B2y-B1y
    Blen = hypot(Bdx, Bdy)
    # if segment is small enough, can maybe ignore its current shape
    if Blen < jumpdistance and Blen < min_edge_len_merging:
        B1x, B1y = Bmx, Bmy
        B2x, B2y = Bmx, Bmy
        Bdx = 0
        Bdy = 0
        Blen = .1 # just to prevent 0-division
    
    # check whether lines are sloped closely enough
    sin_distance1 = abs(Adx*jump_y - Ady*jump_x)/Alen/jumpdistance
    sin_distance2 = abs(Adx*Bdy - Ady*Bdx)/Alen/Blen
    if sin_distance1 > merging_sin_distance_cutoff or\
       sin_distance2 > merging_sin_distance_cutoff:
        #print("merged cancelled bc of different slopes")
        # maybe we can consider corner merge ---- TBD
        return None
    
    
    # find single line fit 
    sigmaAx = Adx*Adx/12
    sigmaAy = Ady*Ady/12
    sigmaAxy = Adx*Ady/12
    sigmaBx = Bdx*Bdx/12
    sigmaBy = Bdy*Bdy/12
    sigmaBxy = Bdx*Bdy/12
    total_len = Alen+Blen
    Cmx = (Alen*Amx + Blen*Bmx) / total_len
    Cmy = (Alen*Amy + Blen*Bmy) / total_len
    var = Alen*Blen / total_len
    Cssx = Alen*sigmaAx + Blen*sigmaBx + var*(Amx-Bmx)*(Amx-Bmx)
    Cssy = Alen*sigmaAy + Blen*sigmaBy + var*(Amy-Bmy)*(Amy-Bmy)
    Csxy = Alen*sigmaAxy + Blen*sigmaBxy + var*(Amx-Bmx)*(Amy-Bmy)
    mx, my, sse = linReg(Cssx, Cssy, Csxy)
    
    # check quality of line fit
    if A_corner: sse += A_hidden_edge*A_hidden_edge/6
    if B_corner: sse += B_hidden_edge*B_hidden_edge/6
    assert sse > -1e-4
    sse = abs(sse)
    rmse = np.sqrt(sse) / total_len
    if rmse > merged_line_rmse_cutoff:
        #print("merge cancelled bc bad fit")
        
        return None
    
    # find start and end points
    if A_corner:
        new_msmt = np.zeros((3,2))
        first_point = segA[0,:2]
        new_msmt[0] = first_point
        # point on fitted line that makes right angle with first_point
        projection = (first_point[0]-Cmx)*mx + (first_point[1]-Cmy)*my
        new_msmt[1,0] = Cmx + projection * mx
        new_msmt[1,1] = Cmy + projection * my
        # point on fitted line that makes right angle with last point on B
        projection = (B2x-Cmx)*mx + (B2y-Cmy)*my
        new_msmt[2,0] = Cmx + projection * mx
        new_msmt[2,1] = Cmy + projection * my
    elif B_corner:
        new_msmt = np.zeros((3,2))
        last_point = segB[2,:2]
        new_msmt[2] = last_point
        # point on fitted line that makes right angle with last_point
        projection = (last_point[0]-Cmx)*mx + (last_point[1]-Cmy)*my
        new_msmt[1,0] = Cmx + projection * mx
        new_msmt[1,1] = Cmy + projection * my
        # point on fitted line that makes right angle with first point on A
        projection = (A1x-Cmx)*mx + (A1y-Cmy)*my
        new_msmt[0,0] = Cmx + projection * mx
        new_msmt[0,1] = Cmy + projection * my
    else:
        # this is just a line segment
        new_msmt = np.zeros((2,2))
        # point on fitted line that makes right angle with first point on A
        projection = (A1x-Cmx)*mx + (A1y-Cmy)*my
        new_msmt[0,0] = Cmx + projection * mx
        new_msmt[0,1] = Cmy + projection * my
        # point on fitted line that makes right angle with last point on B
        projection = (B2x-Cmx)*mx + (B2y-Cmy)*my
        new_msmt[1,0] = Cmx + projection * mx
        new_msmt[1,1] = Cmy + projection * my
        
    # correction for ccw
    if new_msmt[0,0]*new_msmt[-1,1] - new_msmt[0,1]*new_msmt[-1,0] < 0:
        new_msmt = new_msmt[::-1]
    return new_msmt
        
    
gap_angle_to_segment = .03 # radians
gap_distance_to_occlude = .4 # meters, longitudinal
line_fit_quality_cutoff = .5 # meters, rmse
def segmentPoints(points, segments, merge_counter, occlusion_map):
    if len(points) == 2:
        edges = points
    else: # use linear regression to find optimal line segment fit
        min_corner, rmse = getBestFit(points)
        if rmse > line_fit_quality_cutoff: # if no good line fits, don't use current points
            print("segment deleted!!")
    #        occlusion_map.append((atan2(points[0,1], points[0,0]), 1e10,
    #                                hypot(points[0,1], points[0,0])))
    #        occlusion_map.append((atan2(points[-1,1], points[-1,0]),
    #                                hypot(points[-1,1], points[-1,0]), 1e10))
            return merge_counter
        edges = getCorners(points, min_corner)
        
    if edges[0,0]*edges[-1,1] - edges[0,1]*edges[-1,0] < 0:
        edges = edges[::-1]
        
    segment = np.zeros((edges.shape[0], 4))
    segment[:,:2] = edges
    
    if len(segments) == 0:
        # the correct thing to do would be to check the previous rotation
        # we're just ignoring that round here...
        segment[0,2] = True
        segments.append(segment)
        occlusion_map.append((atan2(segment[0,1], segment[0,0]), 1e10,
                                hypot(segment[0,1], segment[0,0])))
        return merge_counter
    
    # figure out merge and occlusion status by looking at other dudes
    last_segment = segments[-1]
    first_edge = segment[0]
    last_rect_edge = last_segment[-1]
    first_edge_dist = hypot(first_edge[0], first_edge[1])
    last_edge_dist = hypot(last_rect_edge[0], last_rect_edge[1])
    relative_distance = first_edge_dist - last_edge_dist
#    # actually sin distance, but nearly identical for significant cases
#    radian_distance = first_edge[1]*last_rect_edge[0]-first_edge[0]*last_rect_edge[1]
#    radian_distance = radian_distance / first_edge_dist / last_edge_dist
    first_edge_angle = atan2(first_edge[1], first_edge[0])
    last_edge_angle = atan2(last_rect_edge[1], last_rect_edge[0])
    radian_distance = (first_edge_angle - last_edge_angle) % (np.pi*2)
    backflip = radian_distance > np.pi*2-.1
    #if backflip: print("backflip")
    
    if not backflip and radian_distance > gap_angle_to_segment:
        # measurements are far apart
        # therefore, both are closed with no chance of occlusion or merging
        last_segment[-1,2] = True
        segment[0,2] = True
        segments.append(segment)
        occlusion_map.append((last_edge_angle, last_edge_dist, 1e10))
        occlusion_map.append((first_edge_angle, 1e10, first_edge_dist))
        return merge_counter

    if not backflip:
        occlusion_map.append((last_edge_angle, last_edge_dist, first_edge_dist))
    else:
        if abs(last_edge_dist - first_edge_dist) > 1.:
            occlusion_map.append((first_edge_angle, last_edge_dist, first_edge_dist))

    # these segments are close, one may occlude the other - or they may be merged
    if abs(relative_distance) < gap_distance_to_occlude:
        #print("merging from gap")
        # to close longitudinally to consider occluding
        # try to merge the segment
        if last_segment[0,3] > 0:
            # last segment was already merged
            # find the background segment - we will merge to this
            merge_situation_id = last_segment[0,3]
            found = False
            for segment_to_merge_idx in xrange(len(segments)-1,-1,-1):
                segment_to_merge = segments[segment_to_merge_idx]
                if segment_to_merge[0,3] == merge_situation_id and\
                        segment_to_merge[1,3]:
                    found = True
                    break
            assert found
        else:
            segment_to_merge_idx = len(segments)-1
            segment_to_merge = segments[segment_to_merge_idx]
            
        merged_segment = mergeSegments(segment_to_merge, segment)
        
        if merged_segment is not None:
            # can successfully merge these segments
            new_merged_segment = np.zeros((merged_segment.shape[0],4))
            new_merged_segment[:,:2] = merged_segment
            new_merged_segment[0,2] = segment_to_merge[0,2] # open-ness
            new_merged_segment[1,3] = True # merge id
            segment[0,2] = True # closed, hits another object
            segment[1,3] = False # merge id
            if not last_segment[0,3]:
                # the last segment wasn't already a merge situation
                # need to update to reflect that it is
                last_segment[-1,2] = True # closed, hits the new segment
                last_segment[0,3] = merge_counter
                new_merged_segment[0,3] = merge_counter
                segment[0,3] = merge_counter
                merge_counter += 1
                # add two segments: original new segment and merged
                segments.append(new_merged_segment)
            else:
                # the last segment was already a merge
                # replace it with updated merge
                new_merged_segment[0,3] = segment_to_merge[0,3]
                segment[0,3] = segment_to_merge[0,3]
                segments[segment_to_merge_idx] = new_merged_segment
            segments.append(segment)
            return merge_counter
    
    if relative_distance > 0:
        # old occludes new
        last_rect_edge[2] = True # last rectangle was closed
        
        
        if len(segments) < 2: 
            segments.append(segment)
            return merge_counter
        
        # check for merge with last open segment
        previous_seg_idx = len(segments)-2
        previous_segment = segments[previous_seg_idx]
        still_can_merge = not previous_segment[-1,2]
        
        if still_can_merge:
            # this segment is open, was also occluded by last_segment too
            if previous_segment[0,3]: # part of a merge, look for the background segment
                merge_situation_id = previous_segment[0,3]
                found = False
                for previous_seg_idx in xrange(previous_seg_idx, -1, -1):
                    previous_segment = segments[previous_seg_idx]
                    if previous_segment[0,3] == merge_situation_id and\
                                previous_segment[1,3]:
                        found = True
                        break
                assert found
                
            # edit for practical purposes
            # if these segments are within a small distance, make them meet halfway
            previous_seg_edge = previous_segment[-1]
            distance = hypot(previous_seg_edge[1]-first_edge[1],
                             previous_seg_edge[0]-first_edge[0])
            if distance < .5:
                previous_seg_x = previous_seg_edge[0]-previous_segment[-2,0]
                previous_seg_y = previous_seg_edge[1]-previous_segment[-2,1]
                new_len = distance/hypot(previous_seg_x,previous_seg_y)/2.
                previous_seg_edge[0] += previous_seg_x*new_len
                previous_seg_edge[1] += previous_seg_y*new_len
                previous_seg_edge[2] = True
                seg_x = first_edge[0] - segment[1,0]
                seg_y = first_edge[1] - segment[1,1]
                new_len = distance/hypot(seg_x,seg_y)/2.
                first_edge[0] += seg_x*new_len
                first_edge[1] += seg_y*new_len
                first_edge[2] = True
        
            # edit for practical purposes: thin long slivers are removed
            previous_angle = atan2(previous_seg_edge[1], previous_seg_edge[0])
            previous_dist = hypot(previous_seg_edge[1], previous_seg_edge[0])
            occluded_angle = (first_edge_angle - previous_angle) % (np.pi*2)
            occluded_distance = abs(first_edge_dist - previous_dist)
            still_can_merge = occluded_distance < .5 or occluded_angle > .03
                
        if still_can_merge:
            #print("merging from occlusion")
            # only handling merges that are in the back
            # otherwise, much more complicated
            merged_segment = mergeSegments(previous_segment, segment)
            still_can_merge = merged_segment is not None

        if still_can_merge:
            new_merged_segment = np.zeros((merged_segment.shape[0],4))
            new_merged_segment[:,:2] = merged_segment
            new_merged_segment[0,2] = previous_segment[0,2] # open-ness
            new_merged_segment[1,3] = True # merge id
            segment[1,3] = False # merge id
            if not previous_segment[0,3]:
                # wasn't already a merge situation
                # need to update to reflect that it is
                previous_segment[0,3] = merge_counter
                new_merged_segment[0,3] = merge_counter
                segment[0,3] = merge_counter
                merge_counter += 1
                # add two segments: original new segment and merged
                segments.append(new_merged_segment)
            else:
                # the previous segment was already a merge
                # update it
                new_merged_segment[0,3] = previous_segment[0,3]
                segment[0,3] = previous_segment[0,3]
                segments[-2] = new_merged_segment
    else:
        # new occludes old
        first_edge[2] = True # this rectangle is closed
        
    segments.append(segment)
    return merge_counter




if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    # make fake lines
    rrr = np.array([[5.,-2],[3,0],[5,2]])
    qqq = np.array([[-5.,2],[-3,0],[-5,-2]])
    points = np.empty((34,2))
    points[:7,0] = np.linspace(rrr[0,0], rrr[1,0], 7, endpoint=False)
    points[:7,1] = np.linspace(rrr[0,1], rrr[1,1], 7, endpoint=False)
    points[7:15,0] = np.linspace(rrr[1,0], rrr[2,0], 8, endpoint=True)
    points[7:15,1] = np.linspace(rrr[1,1], rrr[2,1], 8, endpoint=True)
    points[15:22,0] = np.linspace(qqq[0,0], qqq[1,0], 7, endpoint=False)
    points[15:22,1] = np.linspace(qqq[0,1], qqq[1,1], 7, endpoint=False)
    points[22:30,0] = np.linspace(qqq[1,0], qqq[2,0], 8, endpoint=True)
    points[22:30,1] = np.linspace(qqq[1,1], qqq[2,1], 8, endpoint=True)
    points[30:] = [[-.49,-5],[0,-5],[.49,-5],[10,-10]]
    points[9,0] += .1
    include = np.ones((points.shape[0],), dtype=bool)
    include[10] = False
    points = points[include]
    points[10:14,0] += .3
    points[10:14,1] -= .3
    
    plt.figure()
    #plt.plot(rrr[:,0],rrr[:,1], 'b-')
    #plt.plot(qqq[:,0],qqq[:,1], 'b-')
    plt.plot(points[:,0],points[:,1],'ko')
    
    
    pts = []
    segs = []
    merge_counter = 1
    addPoint(points[0], pts)
    for k in range(1,len(points)):
#        print k
#        print pts
#        print segs
        newseg = addPoint(points[k], pts)
        if newseg is not None:
            merge_counter = segmentPoints(newseg, segs, merge_counter)
    
    for rect in segs:
        plt.plot(rect[:,0], rect[:,1], 'b-')