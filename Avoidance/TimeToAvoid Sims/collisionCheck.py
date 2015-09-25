# -*- coding: utf-8 -*-
"""
Checks for overlap between two vehicles
last modified 9/24/15
"""
import numpy as np

# assume veh = list of numpy arrays, column 1 = x, column 2 = y, column 3 = angle
# vehicle point is front and center
# vehSize = (length, width)
ix = 0
iy = 1
ia = 2
ilength = 0
iwidth = 1

def realign(veh, reference):
    # returns the shifted and rotated version of this rectangle
    # as if other were at the origin with angle 0
    tempx = veh[ix] - reference[ix]
    tempy = veh[iy]- reference[iy]
    tempAng = veh[ia] - reference[ia]
    relativeDist = np.power((np.power(tempx,2) + np.power(tempy,2)),.5)
    relativeAng = np.arctan2(tempy,tempx)
    rotationAng = relativeAng - reference[ia]
    return (relativeDist * np.cos(rotationAng),
            relativeDist * np.sin(rotationAng), tempAng)

def getCorners(rect,size):
    # returns four corners of rectangle
    # assuming given coordinate is front-and-center of rectangle
    # and angle=0 is facing +x (counterclockwise positive)
    x = rect[ix]
    y = rect[iy]
    ang = rect[ia]
    length = size[ilength]
    width = size[iwidth]
    cornersX = (x - width/2 * np.sin(ang),  # x coordinates
            x + width/2 * np.sin(ang),
            x - length*np.cos(ang) - width/2 * np.sin(ang),
            x - length*np.cos(ang) + width/2 * np.sin(ang))
    cornersY = (y + width/2 * np.cos(ang),    # y coordinates
            y - width/2 * np.cos(ang),
            y - length*np.sin(ang) + width/2 * np.cos(ang),
            y - length*np.sin(ang) - width/2 * np.cos(ang))
    return (cornersX, cornersY)

def outside(corners, rect):
    # returns true if these corners are past the lines
    # that define this rectangle
    # if true, then definitely not overlapping
    length = rect[ilength]
    width = rect[iwidth]
    cornersX = np.array(corners[ix])
    cornersY = np.array(corners[iy])
    allleft =  np.all(cornersY > width/2, 0)
    allright = np.all(cornersY < -width/2, 0)
    allfront = np.all(cornersX > 0, 0)
    allbehind =  np.all(cornersX < -length, 0)
    allAll = np.array((allleft, allright, allfront, allbehind))
    return np.any(allAll, 0)
    
def check(veh1, veh2, size1, size2):
    # returns true if there is a collision
    view1 = outside(getCorners(realign(veh2,veh1),size2), size1)
    view2 = outside(getCorners(realign(veh1,veh2),size1), size2)
    return np.logical_not(np.any(np.array((view1, view2)), 0))