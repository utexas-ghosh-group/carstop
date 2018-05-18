#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 2/26/18
"""
import numpy as np
import scipy.optimize as spopt
from scipy.stats import norm as spnorm

def _euclideanDist(X, Y): return np.linalg.norm(X-Y, 2)

def froggerDist(X, Y):
    if X[0] == Y[0]:
        return np.sqrt((X[1] - Y[1])**2 + (X[2] - Y[2])**2)
    return np.inf

# standard way to score tracking
def OSPA(X, Y, p=1, c=100, costFun = _euclideanDist):
    m = len(X)
    n = len(Y)
    if m > n:
        return OSPA(Y, X, p, c, costFun)
    if m == 0:
        return 0 if n==0 else c
    costs = np.array([[costFun(Xi , Yj) for Yj in Y] for Xi in X])
    costs = np.minimum(costs, c) ** p
    row_ind, col_ind = spopt.linear_sum_assignment(costs)
    score = np.sum(costs[row_ind, col_ind]) + c**p * (n-m)
    return (score/n)**(1./p)
    
# "Generalized optimal sub-pattern assignment metric"
# Rahmathullah et al 2017
def GOSPA(X, Y, p=1, c=100, costFun = _euclideanDist):
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
    
# did this myself - like RMS over time
def normalizeGOSPA(ospas, ns, smoothval, p=1):
    ospas = np.convolve(ospas, [1]*smoothval, 'valid')[::smoothval]
    counts = np.convolve(ns, [1]*smoothval, 'valid')[::smoothval]
    return (ospas / counts) ** (1./p)
    
    
def isPSD(matrix): return np.sign(np.linalg.eigvals(matrix))
def isPosDef(matrix): return np.all(np.real(np.linalg.eigvals(matrix)) > 0)

def normpdf(x, prec):
    dev = prec.dot(x).dot(x)
    return (np.linalg.det(prec) / np.exp(dev))**.5 / (2*np.pi)**1.5