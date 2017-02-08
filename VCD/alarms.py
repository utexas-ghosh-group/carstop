# -*- coding: utf-8 -*-
"""
Functions that calculate probability of collision, or make an alarm decision.

vehicle1,vehicle2 = initial_state objects, contain:
    mean, expected, utpoints, sample()
MM1,MM2 = motion model objects, contain:
    ndim, fullToXY() , expected() , sample() , pdf() , UTstep()
times = times at which to check for collision
    
Some models may have additional arguments.
    
returns (
         collision probability or 0-1 guess ,
         time it took to run the significant (repeating) part of the alarm )
"""
from collisionCheck import check as collisionCheck
import numpy as np
import time
import UnscentedTransform as UT

def _forceProb(prob):
    return min(max(prob, 0.), 1.)

def alarm_truth(vehicle1, vehicle2, MM1, MM2, times):
    """ Uses a high-sample MCS alarm to get accurate result. """
    return alarm_MCS(vehicle1, vehicle2, MM1, MM2, times, 20000)
    
def alarm_MCS(vehicle1, vehicle2, MM1, MM2, times, nSamples):
    """ Algorithm 1 in paper """
    starttime = time.time()
    state1 = vehicle1.sample(nSamples)
    state2 = vehicle2.sample(nSamples)
    for dt in times:
        state1 = MM1.sample(state1, dt)
        state2 = MM2.sample(state2, dt)
        collided = collisionCheck(MM1.fullToXY(state1),
                                  MM2.fullToXY(state2))
        state1 = state1[collided==False]
        state2 = state2[collided==False]
    totalCollided = 1 - float(state1.shape[0]) / nSamples
    return ( totalCollided , time.time() - starttime )
    
def alarm_MCS_2(vehicle1, vehicle2, MM1, MM2, times, nSamples):
    """ This version keeps checking samples that already had a collision.
        It's faster on average if collisions are unlikely. """
    starttime = time.time()
    state1 = vehicle1.sample(nSamples)
    state2 = vehicle2.sample(nSamples)
    Collided = np.zeros((nSamples,), dtype=bool) # array set to False
    for dt in times:
        state1 = MM1.sample(state1, dt)
        state2 = MM2.sample(state2, dt)
        collided = collisionCheck(MM1.fullToXY(state1), MM2.fullToXY(state2))
        Collided = Collided | collided
    totalCollided = float(np.sum(Collided)) / nSamples
    return ( totalCollided , time.time() - starttime )
    
    
def alarm_expected(vehicle1, vehicle2, MM1, MM2, times):
    """ Algorithm 2 in paper, where f(X) is expected value"""
    state1 = vehicle1.mean
    state2 = vehicle2.mean
    collided = False
    starttime = time.time()
    for dt in times:
        state1 = MM1.expected(state1, dt)
        state2 = MM2.expected(state2, dt)
        newcollide = collisionCheck(MM1.fullToXY(state1), MM2.fullToXY(state2))
        collided = collided | newcollide
    return ( collided+0. , time.time() - starttime )
    
    
def alarm_UT_1(vehicle1, vehicle2, MM1, MM2, times):
    """ Algorithm 2 in paper, where each f_k(X) is the propagation of one UT
        point. The merge operation is the weighted sum of UT points."""
    points, weights = UT.getUTpoints(MM1.ndim + MM2.ndim)
    npts = points.shape[0]
    v1 = points[:,:MM1.ndim]
    v2 = points[:,MM1.ndim:]
    
    starttime = time.time()
    v1 = v1.dot(np.linalg.cholesky(vehicle1.cov)) + np.tile(vehicle1.mean,(npts,1))
    v2 = v2.dot(np.linalg.cholesky(vehicle2.cov)) + np.tile(vehicle2.mean,(npts,1))
    collided = np.zeros((npts,), dtype=bool)
    
    for dtime in times:
        v1 = MM1.expected(v1, dtime)
        v2 = MM2.expected(v2, dtime)
        xy1 = MM1.fullToXY(v1)
        xy2 = MM2.fullToXY(v2)
        newcollide = collisionCheck(xy1,xy2)
        collided = collided | newcollide
    totalCollided = np.sum(weights[collided])
    totalCollided = _forceProb(totalCollided)
    return ( totalCollided , time.time() - starttime )
    
def alarm_UT_2(vehicle1, vehicle2, MM1, MM2, times):
    """
    A different take on using the unscented transform for collision detection.
    This is not mathematically valid (it completely screws up the
    time-dependency) but gets much better results. If allowed, I will add this
    model into the paper before submission - not sure how that works.
    """
    npoints = vehicle1.utpoints.shape[0]
    weights = np.tile(MM1.weights, (MM2.weights.shape[0],)) *\
              np.repeat(MM2.weights, MM1.weights.shape[0])
    
    starttime = time.time()
    v1 = vehicle1.utpoints
    v2 = vehicle2.utpoints
    collided = np.zeros((npoints**2,), dtype=bool)
    
    for dtime in times:
        v1 = MM1.UTstep(v1, dtime)
        v2 = MM2.UTstep(v2, dtime)
        xy1 = MM1.fullToXY(v1)
        xy2 = MM2.fullToXY(v2)
        xy1 = np.tile(xy1, (npoints,1))
        xy2 = np.repeat(xy2, npoints, 0)
        newcollide = collisionCheck(xy1,xy2)
        collided = collided | newcollide
    totalCollided = np.sum(weights[collided])
    totalCollided = _forceProb(totalCollided)
    return ( totalCollided , time.time() - starttime )