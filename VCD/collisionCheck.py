# -*- coding: utf-8 -*-
"""
Detects overlap of two rectangles.
Uses one fast center-distance check, then a complete check based on realigning
each rectangle horizontally.
The (x,y) coordinate that defines each rectangle is assumed to be
front-and-center, not absolute center!
"""
from numpy import array, sin, cos, arctan2, transpose, all, tile

Vlen = 5.
Vwid = 2.

def realign(veh, reference):
    """ returns the shifted and rotated version of this rectangle
        as if reference were at the origin with angle 0 """
    tempx = veh[:,0] - reference[:,0]
    tempy = veh[:,1] - reference[:,1]
    tempAng = veh[:,2] - reference[:,2]
    relativeDist = pow((pow(tempx,2) + pow(tempy,2)),.5)
    relativeAng = arctan2(tempy,tempx)
    rotationAng = relativeAng - reference[:,2]
    return transpose([relativeDist * cos(rotationAng),
                      relativeDist * sin(rotationAng), tempAng])

def getCorners(rect):
    """ returns four corners of rectangle
        assuming given coordinate is front-and-center of rectangle """
    x = rect[:,0]
    y = rect[:,1]
    ang = rect[:,2]
    length = Vlen
    width = Vwid
    cornersX = array([x + width/2 * sin(ang),
                x - width/2 * sin(ang),
                x - length*cos(ang) + width/2 * sin(ang),
                x - length*cos(ang) - width/2 * sin(ang)])
    cornersY = array([y - width/2 * cos(ang),
                y + width/2 * cos(ang),
                y - length*sin(ang) - width/2 * cos(ang),
                y - length*sin(ang) + width/2 * cos(ang)])
    return [cornersX, cornersY]

def outside(corners):
    """ returns true if these corners are past the lines that define the
        rectangle at (angle 0, coord (0,0));
        if true, then definitely not overlapping """
    cornersX, cornersY = corners
    allleft =  all(cornersX < -Vlen, axis=0)
    allright = all(cornersX > 0, axis=0)
    allabove = all(cornersY > Vwid/2., axis=0)
    allbelow =  all(cornersY < -Vwid/2., axis=0)
    return allleft | allright | allabove | allbelow
    
def check(veh1, veh2):
    """ checks for overlap of rectangles"""
    if veh1.ndim == 1:
        veh1 = tile(veh1, (1,1))
        veh2 = tile(veh2, (1,1))
        answer = _check(veh1, veh2)
        return answer[0]
    return _check(veh1, veh2)
    
def _check(veh1, veh2):
    results = array([False]*len(veh1))
    ## fast check first
    longestConnection = (Vlen**2 + Vwid**2/4.) * 4.
    pointdistance = (veh1[:,0]-veh2[:,0])**2 + (veh1[:,1]-veh2[:,1])**2
    closeenough = pointdistance <= longestConnection
    veh1 = veh1[closeenough,:]
    veh2 = veh2[closeenough,:]
    ## now slow check, realigning each vehicle to center
    view1 = outside(getCorners(realign(veh2,veh1)))
    view2 = outside(getCorners(realign(veh1,veh2)))
    results[closeenough] = (view1==False) & (view2==False)
    return results