#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 5/10/18
simple Kalman filter with hardcoded parameters
derives speed, and also corrects motion estimates a little
"""
import numpy as np
#import scipy.optimize as spopt
#from scipy.spatial import distance_matrix as distanceMatrix
#import utm


gps_delt = .2
gps_R = 2. ** 2 # variance of each position measurement
gps_Qx = .5 ** 2 * gps_delt # random motion
gps_Qv = 2.5 ** 2 * gps_delt # random change in speed
gps_init_Px = 2. ** 2
gps_init_Pv = 30. ** 2


class singleGpsTracker():
    def __init__(self, coord):
        self.pos = np.array(coord)#utm.from_latlon(coord[0], coord[1])[:2]
        self.speed = np.array([0.,0.])
        self.covs_pos = np.array([gps_init_Px]*2)
        self.covs_ps = np.zeros((2,))
        self.covs_speed = np.array([gps_init_Pv]*2)
        #self.last_time = time
		
    def predict(self):
        delt = .2
        #assert delt >= .1999
        #self.last_time = time
        self.pos += self.speed * delt
        self.covs_pos += self.covs_ps * delt * 2 +\
                         self.covs_speed * delt**2 + gps_Qx
        self.covs_ps += self.covs_speed * delt
        self.covs_speed += gps_Qv
	
    def update(self, coord):
        if coord == None:
            pass
        else:
            coord = np.array(coord)#utm.from_latlon(coord[0], coord[1])[:2]
            deviation = coord - self.pos
            precision = 1. / (self.covs_pos + gps_R)
            self.pos += deviation * precision * self.covs_pos
            self.speed += deviation * precision * self.covs_ps
            self.covs_speed -= self.covs_ps**2 * precision
            multiplier = 1 - self.covs_pos * precision
            self.covs_ps *= multiplier
            self.covs_pos *= multiplier



#class GpsTracker():
#    def __init__(self):
#        self.pos = []
#        self.speed = []
#        self.covs_pos = []
#        self.covs_speed = []
#        self.covs_ps = []
#        self.ids = []
#        self.used_ids = 0
#        
#    def update(self, coords):          
#        # predict
#        for k in range(len(self.pos)):
#            self.pos[k] += self.speed[k] * gps_delt
#            self.covs_pos[k] += self.covs_ps[k] * gps_delt * 2 +\
#                                self.covs_speed[k] * gps_delt**2 + gps_Qx
#            self.covs_ps[k] += self.covs_speed[k] * gps_delt
#            self.covs_speed[k] += gps_Qv
#            
#        # data association
#        utm_coords = coords
#        #utm_coords = [np.array(utm.from_latlon(wsm_coord[0], wsm_coord[1])[:2])
#        #                for wsm_coord in coords]
#        if len(self.pos) == 0:
#            cost_matrix = np.zeros((len(utm_coords), 0))
#        elif len(utm_coords) == 0:
#            cost_matrix = np.zeros((0, len(self.pos)))
#        else:
#            cost_matrix = distanceMatrix(utm_coords, self.pos)
#        miss_matrix = np.zeros(cost_matrix.shape) + 10 # can't jump 20 meters
#        miss_idxs = cost_matrix > miss_matrix
#        cost_matrix = np.minimum(miss_matrix, cost_matrix)
#        msmt_match, obj_match = spopt.linear_sum_assignment(cost_matrix)
#        assert np.sum(cost_matrix[msmt_match, obj_match]) < 45
#        missed_matches = miss_idxs[msmt_match, obj_match]
#        missed_measurements = [msmt_idx for msmt_idx in
#                               range(len(utm_coords)) if not (msmt_idx in msmt_match)]
#        
#        # update
#        new_pos = []
#        new_speed = []
#        new_covs_pos = []
#        new_covs_speed = []
#        new_covs_ps = []
#        new_ids = []
#        for msmt_idx in missed_measurements:
#            new_pos.append( utm_coords[msmt_idx] )
#            new_speed.append( np.zeros((2,)) )
#            new_covs_pos.append( np.array([gps_init_Px]*2) )
#            new_covs_speed.append( np.array([gps_init_Pv]*2) )
#            new_covs_ps.append( np.zeros((2,)) )
#            new_ids.append( self.used_ids )
#            self.used_ids += 1
#        for msmt_idx, obj_idx, miss in zip(msmt_match, obj_match, missed_matches):
#            if miss: # new car in view
#                new_pos.append( utm_coords[msmt_idx] )
#                new_speed.append( np.zeros((2,)) )
#                new_covs_pos.append( np.array([gps_init_Px]*2) )
#                new_covs_speed.append( np.array([gps_init_Pv]*2) )
#                new_covs_ps.append( np.zeros((2,)) )
#                new_ids.append( self.used_ids )
#                self.used_ids += 1
#                
#            else:
#                # Kalman filter
#                deviation = utm_coords[msmt_idx] - self.pos[obj_idx]
#                precision = 1. / (self.covs_pos[obj_idx] + gps_R)
#                self.pos[obj_idx] +=\
#                            deviation * precision * self.covs_pos[obj_idx]
#                self.speed[obj_idx] +=\
#                             deviation * precision * self.covs_ps[obj_idx]
#                self.covs_speed[obj_idx] -=\
#                                   self.covs_ps[obj_idx]**2 * precision
#                multiplier = precision * (1 - self.covs_pos[obj_idx])
#                self.covs_ps[obj_idx] *= multiplier
#                self.covs_pos[obj_idx] *= multiplier
#                
#        self.pos = [self.pos[obj_idx] for obj_idx in obj_match] + new_pos
#        self.speed = [self.speed[obj_idx] for obj_idx in obj_match] + new_speed
#        self.covs_pos = [self.covs_pos[obj_idx] for obj_idx in obj_match] + new_covs_pos
#        self.covs_speed = [self.covs_speed[obj_idx] for obj_idx in obj_match
#                           ] + new_covs_speed
#        self.covs_ps = [self.covs_ps[obj_idx] for obj_idx in obj_match] + new_covs_ps
#        self.ids = [self.ids[obj_idx] for obj_idx in obj_match] + new_ids
#            
#        # return
#        total_speed = [np.hypot(speed[0], speed[1]) for speed in self.speed]
#        return zip(self.pos, total_speed, self.ids)