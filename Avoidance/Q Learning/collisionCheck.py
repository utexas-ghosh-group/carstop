# -*- coding: utf-8 -*-
"""
Checks for overlap between two vehicles
last modified 4/20/16, specifically for Q learning
"""
from pandas import Series
from math import sin, cos, atan2, pi

collisionVars = ['x','y','angle','length','width']

def realign(veh, reference):
    # returns the shifted and rotated version of this rectangle
    # as if other were at the origin with angle 0
    tempx = veh.x - reference.x
    tempy = veh.y - reference.y
    tempAng = veh.angle - reference.angle
    relativeDist = pow((pow(tempx,2) + pow(tempy,2)),.5)
    relativeAng = atan2(tempy,tempx)
    rotationAng = relativeAng - reference.angle
    return Series([relativeDist * cos(rotationAng),
                   relativeDist * sin(rotationAng),
                   tempAng, veh.length, veh.width], index = collisionVars)

def getCorners(rect):
    # returns four corners of rectangle
    # assuming given coordinate is front-and-center of rectangle
    # and angle=0 is facing -y (this is SUMO's convention)
    x = rect.x
    y = rect.y
    ang = rect.angle
    length = rect.length
    width = rect.width
    cornersX = [x + width/2 * cos(ang),
            x - width/2 * cos(ang),
            x - length*sin(ang) + width/2 * cos(ang),
            x - length*sin(ang) - width/2 * cos(ang)]
    cornersY = [y + width/2 * sin(ang),
            y - width/2 * sin(ang),
            y + length*cos(ang) + width/2 * sin(ang),
            y + length*cos(ang) - width/2 * sin(ang),]
    return [cornersX, cornersY]

def outside(rect, corners):
    # returns true if these corners are past the lines
    # that define this rectangle
    # if true, then definitely not overlapping
    allleft =  max(corners[0]) < -rect.width/2
    allright = min(corners[0]) > rect.width/2
    allabove = min(corners[1]) > rect.length
    allbelow =  max(corners[1]) < 0
    return (allleft or allright or allabove or allbelow)
    
def longestArc(rect):
    # maximum distance something can be from rectangle's coordinate
    # to overlap with rectangle
    return pow(pow(rect.length,2) + pow(rect.width,2)/4, .5)
    
def check(veh1, veh2):
    # fast check first
    longestConnection = longestArc(veh1) + longestArc(veh2)
    if abs(veh1.x - veh2.x) > longestConnection:
        return False
    if abs(veh1.y - veh2.y) > longestConnection:
        return False
    # switch to SUMO angles because I'm too lazy to fix everything
    veh1.angle = veh1.angle + pi/2
    veh2.angle = veh2.angle + pi/2
    # now slow check
    view1 = outside(veh1,getCorners(realign(veh2,veh1)))
    view2 = outside(veh2,getCorners(realign(veh1,veh2)))
    return not (view1 or view2)