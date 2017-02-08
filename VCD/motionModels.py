# -*- coding: utf-8 -*-
"""
Classes that predict vehicle's current and future position.

Two state formats:
    full - the entire state, determines motion (along with any fixed parameters)
    XY - [x,y,angle] used to check for collisions
    
functions in each MM (Motion Model) class:
    fullToXY
    expected - return expected value of state at a time
    sample - return independent random samples of full state after motion
    UTstep - take the expected value a set of unscented points over a timestep,
             then compute the resulting distribution and the next set of
             unscented points

functions in each initialState class:
    mean - the expected position of the vehicle
    utpoints - the unscented transform points for that
    sample() - return independent random samples of full state at current time
"""
import numpy as np
from scipy.stats import multivariate_normal as rmvnorm
from roadLoc import roadLoc
import UnscentedTransform as UT
        

class MM_LineCV():
    """ Vehicle follows a predetermined road. The roads are based on an
        intersection in SUMO, as explained in the roadLoc module.
        
        state = [displacement, velocity]
        noiseMatrix = per-second variance in motion equation """
        
    def __init__(self, route, noiseMatrix):
        self.cov = noiseMatrix
        self.route = route
        points, weights = UT.getUTpoints(2, 1.)
        self.points = points
        self.weights = weights
        self.ndim = 2
        
    def fullToXY(self, state):
        if state.ndim == 1:
            return np.array(roadLoc(state[0], self.route))
        else:
            return roadLoc(state[:,0], self.route)
        
    def expected(self, state, time):
        if state.ndim == 1:
            return np.array((state[0]+state[1]*time, state[1]))
        else:
            return np.array((state[:,0]+state[:,1]*time, state[:,1])).T
        
    def sample(self, state, time, nsamples = 1):
        vv = self.expected(state, time)
        if vv.ndim == 1:
            return vv + rmvnorm.rvs([0,0], self.cov*time, size=nsamples)
        else:
            return vv + rmvnorm.rvs([0,0], self.cov*time, size=vv.shape[0])
        
    def UTstep(self, inpoints, time):
        points = self.expected(inpoints, time)
        mean = np.tile(self.weights.dot(points), (points.shape[0],1))
        points = points - mean
        cov = points.T.dot(np.diag(self.weights)).dot(points) + self.cov*time
        return mean + self.points.dot(np.linalg.cholesky(cov).T)

        
class MM_Bicycle():
    """
    The bicycle model has been used a lot for car motion, a few papers defining
    it were cited in the paper.
    
    state = [x, y, angle, velocity, acceleration, angular velocity]
    noiseMatrix = covariance matrix of motion noise (for roughly 1 second)
    """
    
    def __init__(self, noiseMatrix):
        self.cov = noiseMatrix
        points, weights = UT.getUTpoints(6, 1.)
        self.points = points
        self.weights = weights
        self.ndim = 6
        
    def fullToXY(self, state):
        if state.ndim==1:
            return state[:3]
        else:
            return state[:,:3]
        
    def expected(self, state, time):
        if state.ndim == 1:
            return np.array((state[0]+np.cos(state[2])*state[3]*time,
                             state[1]+np.sin(state[2])*state[3]*time,
                             state[2]+state[5]*time, state[3]+state[4]*time,
                             state[4], state[5]))
        else:
            return np.array((state[:,0]+np.cos(state[:,2])*state[:,3]*time,
                             state[:,1]+np.sin(state[:,2])*state[:,3]*time,
                             state[:,2]+state[:,5]*time,
                             state[:,3]+state[:,4]*time,
                             state[:,4], state[:,5])).T
            
    def sample(self, state, time, nsamples = 1):
        vv = self.expected(state, time)
        if vv.ndim == 1:
            return vv + rmvnorm.rvs([0.]*6, self.cov*time, size=nsamples)
        else:
            return vv + rmvnorm.rvs([0.]*6, self.cov*time, size=vv.shape[0])
            
    def UTstep(self, inpoints, time):
        points = self.expected(inpoints, time)
        mean = np.tile(self.weights.dot(points), (points.shape[0],1))
        # compute angle mean properly
        cosmean = self.weights.dot(np.cos(points[:,2]))
        sinmean = self.weights.dot(np.sin(points[:,2]))
        mean[:,2] = np.arctan2(sinmean, cosmean)
        #
        points = points - mean
        # reformat angles to be in +-pi
        angles = points[:,2]
        angles[angles > np.pi] = angles[angles > np.pi] - np.pi*2
        angles[angles < -np.pi] = angles[angles < -np.pi] + np.pi*2
        points[:,2] = angles
        #
        cov = points.T.dot(np.diag(self.weights)).dot(points) + self.cov*time
        outpoints = mean + self.points.dot(np.linalg.cholesky(cov).T)
        # reformat angles to be in +-pi
        angles = outpoints[:,2]
        angles[angles > np.pi] = angles[angles > np.pi] - np.pi*2
        angles[angles < -np.pi] = angles[angles < -np.pi] + np.pi*2
        outpoints[:,2] = angles
        return outpoints
        
            
        
class initialState_normal():
    """ The vehicle's current state X_0 is normally distributed. """
    
    def __init__(self, mean, cov):
        self.mean = mean
        self.cov = cov
        chol = np.linalg.cholesky(cov).T
        utpoints, utweights = UT.getUTpoints(mean.shape[0])
        self.utpoints = np.tile(mean,(utpoints.shape[0],1)) + utpoints.dot(chol)
    def sample(self, nsamples):
        return rmvnorm.rvs(self.mean, self.cov, size=nsamples)