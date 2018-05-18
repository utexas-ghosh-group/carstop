#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 5/17/18
"""
import scipy.optimize as spopt
import numpy as np

def overlap(a, b):
    xdiff = a[0]-b[0]
    ydiff = a[1]-b[1]
    return b[2] > xdiff and a[2] > -xdiff and b[3] > ydiff and a[3] > -ydiff

def IoU(a, b):
    if not overlap(a,b): return 0
    areaA = a[2]*a[3]
    areaB = b[2]*b[3]
    I = (min(a[0]+a[2], b[0]+b[2]) - max(a[0], b[0])) *\
        (min(a[1]+a[3], b[1]+b[3]) - max(a[1], b[1]))
    return I / (areaA + areaB - I)

def minusIoU(a, b): return 1 - IoU(a, b)

def GOSPA(X, Y, p=1, c=1., costFun = minusIoU):
    m = len(X)
    n = len(Y)
    if m > n:
        return GOSPA(Y, X, p, c, costFun)
    if m == 0:
        return c**p / 2. * n
    costs = np.array([[costFun(Xi , Yj) for Yj in Y] for Xi in X])
    costs = np.minimum(costs, c) ** p
    row_ind, col_ind = spopt.linear_sum_assignment(costs)
    return np.sum(costs[row_ind, col_ind]) + c**p / 2. * (n-m)