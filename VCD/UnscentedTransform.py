#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Chooses the points and weights for a 3-layer unscented transform.

This variant of the transform uses O(n^2) points for n dimensions but captures
motion up to the fourth derivative, assuming the initial distribution is normal.

The equations for choosing a 3-layer unscented transform's parameters are
described in "A Consistent, Debiased Method for Converting Between Polar and
Cartesian Coordinate Systems", Julier & Uhlmann '97.
"""
import numpy as np

def getUTpoints(n, s1 = 1.):
    """ Generates points and weights
        
        n = number of dimensions
        s1 = distance of the points along each axis """
    s2, w0, w1, w2 = getUTparams(n, s1)
    points = np.zeros((2*n*(n-1)+2*n+1, n))
    for i in range(n):
        points[i*2+1 , i] = s1
        points[i*2+2 , i] = -s1
        for j in range(i):
            idx = i*(i-1)*2 + j*4 + n*2
            points[idx+1, i] = s2
            points[idx+1, j] = s2
            points[idx+2, i] = s2
            points[idx+2, j] = -s2
            points[idx+3, i] = -s2
            points[idx+3, j] = s2
            points[idx+4, i] = -s2
            points[idx+4, j] = -s2
    weights = np.array([w0]+[w1]*2*n+[w2]*2*n*(n-1))
    return points, weights
    
def getUTparams(n, s1):
    """ There are five parameters and only four conditions, so one parameter
        must be fixed, in this case s1."""
    assert n > 1 , "one-dimensional UT transform not implemented"
    w1 = (2. - n) / 2 / s1**2
    s2 = ((3. - (2 - n)*s1**2) / (n - 1)) ** .5
    w2 = 1. / 4 / s2**2
    w0 = 1. - 2*n*w1 - 2*n*(n-1)*w2
    return (s2, w0, w1, w2)