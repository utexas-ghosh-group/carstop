# -*- coding: utf-8 -*-
"""
4/5/16
better: scipy.spatial.ConvexHull
"""
from numpy import argmin
import matplotlib.pyplot as pyplot

#I took this from some website
def convex_hull_2d(points):
    """Computes the convex hull of a set of 2D points.

    Input: an iterable sequence of (x, y) pairs representing the points.
    Output: a list of vertices of the convex hull in counter-clockwise order,
      starting from the vertex with the lexicographically smallest coordinates.
    Implements Andrew's monotone chain algorithm. O(n log n) complexity.
    """

    # Sort the points lexicographically (tuples are compared lexicographically).
    # Remove duplicates to detect the case we have just one unique point.
    points = sorted(set(points))

    # Boring case: no points or a single point, possibly repeated multiple times.
    if len(points) <= 1:
        return points

    # 2D cross product of OA and OB vectors, i.e. z-component of their 3D cross product.
    # Returns a positive value, if OAB makes a counter-clockwise turn,
    # negative for clockwise turn, and zero if the points are collinear.
    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    # Build lower hull 
    lower = []
    for p in points:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    # Build upper hull
    upper = []
    for p in reversed(points):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    # Concatenation of the lower and upper hulls gives the convex hull.
    # Last point of each list is omitted because it is repeated at the beginning of the other list. 
    return lower[:-1] + upper[:-1]
    
def vprod(d1, d2):
    return d1[0]*d2[0]+d1[1]*d2[1]        
    
def psin(d1,d2):
    return (vprod(d1,d1)*vprod(d2,d2)-vprod(d1,d2)**2)**.5   
    
def getTriangle(p1, p2, p3, p4):
    d12 = (p2[0]-p1[0],p2[1]-p1[1])
    d23 = (p3[0]-p2[0],p3[1]-p2[1])
    d34 = (p4[0]-p3[0],p4[1]-p3[1])
    eps = 10.**(-10)
    if abs(d12[1]*d23[0]-d12[0]*d23[1]) < eps: # point 2 is in straight line
        return [p3, 0]
    if abs(d34[1]*d23[0]-d34[0]*d23[1]) < eps: # point 3 is in straight line
        return [p2, 0]
    if d34[1]*d12[0]-d34[0]*d12[1] <= eps: # can't make triangle
        return[ None, 10000]
#    psi3 = psin(d23,d12)
#    psi2 = psin(d23,d34)
#    psiMiddle = psin(d34,d12)
    newdist3 = psin(d23,d12)/psin(d34,d12)
    newpoint3 = (p3[0] - d34[0]*newdist3, p3[1] - d34[1]*newdist3)
    newdist2 = psin(d23,d34)/psin(d34,d12)
    newpoint2 = (p2[0] + newdist2*d12[0], p2[1] + newdist2*d12[1])
    return [newpoint2 , psin(d23,d12)*psin(d34,d23)/psin(d34,d12)/2]
    
def simplifyConvexHull2d(points, finalcount = 4):
    # starting with points in cc order:
    npt = len(points)
    areas = []
    potentialPoints = []
    for i in range(npt):
        triangPoints = points[i:min(i+4,npt)] + points[:max(0,i-npt+4)]
        newpoint, area = getTriangle(*triangPoints)
        potentialPoints += [newpoint]
        areas += [area]
    while npt > finalcount and npt > 4 and min(areas) < 10000:
        newpointInd = argmin(areas)
        newpoint = potentialPoints[newpointInd]
        
        extendedPoints = points[-1:]+points+points[:4] # don't have to wrap around
        pointBefore, areaBefore = getTriangle(extendedPoints[newpointInd],
                                              extendedPoints[newpointInd+1],
                                              newpoint,
                                              extendedPoints[newpointInd+4])
        pointAfter, areaAfter = getTriangle(extendedPoints[newpointInd+1],
                                              newpoint,
                                              extendedPoints[newpointInd+4],
                                              extendedPoints[newpointInd+5])
        newPotentialPoint = potentialPoints[(newpointInd+2)%npt]
        potentialPoints[newpointInd-1] = pointBefore
        potentialPoints[newpointInd] = pointAfter
        newArea = areas[(newpointInd+2)%npt]
        areas[newpointInd-1] = areaBefore
        areas[newpointInd] = areaAfter

        # reconfigure arrays    
        newpoints = points[max(0,newpointInd-npt+3):newpointInd+1]
        newpoints += [newpoint]
        newpoints += points[min(newpointInd+3,npt):npt]
        newpotentials = potentialPoints[max(0,newpointInd-npt+3):newpointInd+1]
        newpotentials += [newPotentialPoint]
        newpotentials += potentialPoints[min(newpointInd+3,npt):npt]
        newareas = areas[max(0,newpointInd-npt+3):newpointInd+1]
        newareas += [newArea]
        newareas += areas[min(newpointInd+3,npt):npt]
        points = newpoints
        potentialPoints = newpotentials
        areas = newareas
        
        npt = npt - 1
    return points
        
def getHull(points2d):
    #points2d = list((ainfo[i],binfo[i]) for i in range(len(ainfo)))
    hull = convex_hull_2d(points2d)
    polygon = simplifyConvexHull2d(hull, 4)
    
#    pyplot.plot(list((x for x,y in hull)),
#                list((y for x,y in hull)),'bo',
#                list((x for x,y in polygon)),list((y for x,y in polygon)),'rs')
    return polygon
    
def hull2matrix(points, makepoints = None):
    # set of intersecting hyperplanes written as Ax <= b
    if not makepoints is None:
        points = list(((points[i],makepoints[i]) for i in range(len(points))))
    matrix = []
    intercept = []
    npt = len(points)
    points += [points[0]] # extend for simplicity 
    for ind in range(npt):
        x1,y1 = points[ind]
        x2,y2 = points[ind+1]
        matrix += [[y2-y1,x1-x2]]
        intercept += [x1*y2-x2*y1]
    return [matrix, intercept]
        
def minTTC(hull, u, v, margin=0., safeValue = 1000):
    # x = u*t + v, for scalar t
    # with margins, x = u*t + v +\- margin
    # finds lowest value of t at which x is in the hull
    # the line x never intersects the hull, return safeValue
    normu = (u[0]**2+u[1]**2)**.5
    if normu == 0:
        return safeValue
    if margin > 0:
        vleft = (v[0]-u[1]*margin/normu , v[1]+u[0]*margin/normu)
        vright = (v[0]+u[1]*margin/normu , v[1]-u[0]*margin/normu)
        
    matrix, intercept = hull2matrix(hull)
    lowerBound = 0
    upperBound = 100

    for i in range(len(matrix)):
        row = matrix[i]
        denominator = row[0]*u[0] + row[1]*u[1]
        if denominator == 0:
            pass
        elif denominator > 0: # leq constraint
            check = (intercept[i] - row[0]*v[0] - row[1]*v[1])/ denominator
            if margin > 0:
                margin1check = (intercept[i]-row[0]*vleft[0] - row[1]*vleft[1])/denominator
                margin2check = (intercept[i]-row[0]*vright[0] - row[1]*vright[1])/denominator
                check = max(margin1check, margin2check)
            if check < upperBound:
                upperBound = check
        elif denominator < 0: # geq
            check = (intercept[i] - row[0]*v[0] - row[1]*v[1])/ denominator
            if margin > 0:
                margin1check = (intercept[i]-row[0]*vleft[0] - row[1]*vleft[1])/denominator
                margin2check = (intercept[i]-row[0]*vright[0] - row[1]*vright[1])/denominator
                check = min(margin1check, margin2check)
            if check > lowerBound:
                lowerBound = check
    if upperBound <= lowerBound:
        return safeValue
    return lowerBound