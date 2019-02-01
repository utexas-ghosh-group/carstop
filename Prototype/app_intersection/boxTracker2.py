#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 11/13/18
"""
import numpy as np
import numba as nb

nfeatures = 43
sensor_noise = .4**2
angle_update_rate = .2 # how much new angle is considered compared to old angle
parallel = False

newsample_max_dim = 5.
newsample_min_pos_var = 1. **2
newsample_min_dim_var = 1. **2
newsample_min_speed_var = 4. **2


@nb.jit(nb.void(nb.f8[:], nb.f8[:,:]), nopython=True)
def covAsSquare(sample, cov):
    for k in range(6):
        idx = 7+k*6
        cov[k,:] = sample[idx:idx+6]
        
@nb.jit(nb.void(nb.f8[:], nb.f8[:,:]), nopython=True)
def covAsList(sample, cov):
    for k in range(6):
        idx = 7+k*6
        sample[idx:idx+6] = cov[k,:]


# prep parameters are used to store calculations that would be the same for
# likelihood/updates of object A with measurement 1, and object A with measurement 2
# if these parameters are passed into the functions, parallel must be False
# because every sample is using the same structures
# to run parallel, it will be necessary to make larger structures
# i.e.  paramslist = np.zeros((n,3)); params = paramslist[sample_idx]
params = np.zeros((3,))
prep_meanz = np.zeros((8,))
prep_gain = np.zeros((6,8))
prep_covz = np.zeros((8,8))
prep_cov = np.zeros((6,6))
cos=.99
sin=0
H = np.array([[1, 0, -cos, -sin],
              [0, 1, -sin,  cos],
              [1, 0,  cos, -sin],
              [0, 1,  sin,  cos],
              [1, 0,  cos,  sin],
              [0, 1,  sin, -cos],
              [ cos,  sin, 1, 0],
              [-sin,  cos, 0, 1]])
preps = (params, prep_meanz, prep_gain, prep_covz, prep_cov, H)

# three corners and two lines
# convert angle (and swap length/width) to find direction where origin is in
# first quadrant
# [ 1  0 -c -s]
# [ 0  1 -s  c]
# [ 1  0  c -s]
# [ 0  1  s  c]
# [ 1  0  c  s]
# [ 0  1  s -c]
# [ c  s  1  0]
# [-s  c  0  1]
# skew = [[0,-1],[1,0]]; flip = [[-1,0],[0,-1]]
things_for_H_cos = np.array([[1,3],[2,2],[3,3],[4,2],[6,0],[7,1]], dtype=int)
things_for_H_ncos = np.array([[0,2],[5,3]], dtype=int)
things_for_H_sin = np.array([[3,2],[4,3],[5,2],[6,1]], dtype=int)
things_for_H_nsin = np.array([[0,3],[1,2],[2,3],[7,0]], dtype=int)
@nb.jit(nb.void(nb.f8[:], nb.f8[:], nb.f8[:], nb.f8[:,:], nb.f8[:,:],
                nb.f8[:,:], nb.f8[:,:]),
        nopython=True, parallel=parallel)
def prepObject(sample, params, meanz, kalman_gain, covz, cov, H):
    view_angle = np.arctan2(-sample[2], -sample[1])
    f = np.floor_divide(sample[0] - view_angle, np.pi/2)
    if f%2 == 0:
        sample[3:5] = sample[4:2:-1]
        covAsSquare(sample, cov)
        cov[:,2:4] = cov[:,3:1:-1]
        cov[2:4,:] = cov[3:1:-1,:]
        covAsList(sample, cov)
    sample[0] -= np.pi/2*(f+1)
    
    sin = np.sin(sample[0])
    cos = np.cos(sample[0])
    for thing_for_H in range(things_for_H_cos.shape[0]):
        idx0, idx1 = things_for_H_cos[thing_for_H]
        H[idx0, idx1] = cos
    for thing_for_H in range(things_for_H_ncos.shape[0]):
        idx0, idx1 = things_for_H_ncos[thing_for_H]
        H[idx0, idx1] = -cos
    for thing_for_H in range(things_for_H_sin.shape[0]):
        idx0, idx1 = things_for_H_sin[thing_for_H]
        H[idx0, idx1] = sin
    for thing_for_H in range(things_for_H_nsin.shape[0]):
        idx0, idx1 = things_for_H_nsin[thing_for_H]
        H[idx0, idx1] = -sin
    
    meanz[:] = np.dot(H, sample[1:5])
    covAsSquare(sample, cov)
    kalman_gain[:] = np.dot(cov[:,:4], H.T)
    covz[:] = np.dot(H, kalman_gain[:4,:])
    max_covz = 0.
    for k in range(8):
        covz[k,k] += sensor_noise
        if covz[k,k] > max_covz:
            max_covz = covz[k,k]
    
    # early cutoff - max distance from true value
    max_variance = max_covz * 4
    
    params[0] = cos; params[1] = sin; params[2] = max_variance


@nb.jit(nb.i1[:](nb.f8, nb.f8, nb.f8[:,:], nb.f8[:]),
        nopython=True, parallel=parallel)
def HHH(cos, sin, measurement, measurement_buffer):
    measurement_closed = measurement[:,2]
    measurement = measurement[:,:2]
    if measurement_closed[2] >= 0: # both sides are detected
        if measurement_closed[0] and measurement_closed[2]: # all three corners
            f_measurement = measurement.flatten() 
            include = [0,1,2,3,4,5]
        elif measurement_closed[0]: # left and center corners, right side
            f_measurement = np.empty((5,))
            f_measurement[0:2] = measurement[0]
            f_measurement[2:4] = measurement[1]
            f_measurement[4] = measurement[2,0]*cos + measurement[2,1]*sin
            include = [0,1,2,3,6]
        elif measurement_closed[1]: # center and right corners
            f_measurement = np.empty((5,))
            f_measurement[0:2] = measurement[1]
            f_measurement[2:4] = measurement[2]
            f_measurement[4] = measurement[0,1]*cos - measurement[0,0]*sin
            include = [2,3,4,5,7]
        else:
            f_measurement = np.empty((4,))
            f_measurement[0:2] = measurement[1]
            f_measurement[2] = measurement[1,0]*cos + measurement[1,1]*sin
            f_measurement[3] = measurement[0,1]*cos - measurement[0,0]*sin
            include = [2,3,6,7]
    else: # only one side detected, must determine which
        slope = measurement[1] - measurement[0]
        # cutoff for sides = angle - np.pi/4
        if cos*(slope[0]+slope[1]) > sin*(slope[0]-slope[1]): # left side
            if measurement_closed[0] and measurement_closed[1]: # full segment
                include = [0,1,2,3]
                f_measurement = measurement.flatten()
            elif measurement_closed[0]: # corner on left, open right
                include = [0,1,7]
                f_measurement = np.empty((3,))
                f_measurement[:2] = measurement[0]
                f_measurement[2] = measurement[1,1]*cos - measurement[1,0]*sin
            elif measurement_closed[1]: # corner on right, open left
                include = [2,3,7]
                f_measurement = np.empty((3,))
                f_measurement[:2] = measurement[1]
                f_measurement[2] = measurement[0,1]*cos - measurement[0,0]*sin
            else: # open segment, can use either measurement
                include = [7]
                f_measurement = np.array((measurement[0,1]*cos -
                                          measurement[0,0]*sin,))
        else:
            if measurement_closed[0] and measurement_closed[1]: # full segment
                include = [2,3,4,5]
                f_measurement = measurement.flatten()
            elif measurement_closed[0]: # corner on left, open right
                include = [2,3,6]
                f_measurement = np.empty((3,))
                f_measurement[:2] = measurement[0]
                f_measurement[2] = measurement[1,0]*cos + measurement[1,1]*sin
            elif measurement_closed[1]: # corner on right, open left
                include = [4,5,6]
                f_measurement = np.empty((3,))
                f_measurement[:2] = measurement[1]
                f_measurement[2] = measurement[0,0]*cos + measurement[0,1]*sin
            else: # open segment, can use either measurement
                include = [6]
                f_measurement = np.array((measurement[0,0]*cos +
                                          measurement[0,1]*sin,))
    for i,j in enumerate(include): measurement_buffer[j] = f_measurement[i]
    return np.array(include, dtype=np.int8)

    
_pdf_const = np.log(2*np.pi)
@nb.jit(nb.f8(nb.f8[:], nb.f8[:,:], nb.f8[:], nb.f8[:], nb.f8[:,:], nb.f8[:,:],
              nb.f8[:,:], nb.f8[:,:]),
        nopython=True, parallel=parallel)
def likelihood(sample, measurement, prep_params, prep_meanz, prep_gain,
               prep_covz, prep_cov, prep_H):
    breaker = True
    has2 = measurement[2,2] < 0
    if  np.square(prep_meanz[0] - measurement[0,0]) +\
        np.square(prep_meanz[1] - measurement[0,1]) < prep_params[2]:
            breaker = False
    elif np.square(prep_meanz[0] - measurement[1,0]) +\
         np.square(prep_meanz[1] - measurement[1,1]) < prep_params[2]:
            breaker = False
    elif np.square(prep_meanz[2] - measurement[0,0]) +\
         np.square(prep_meanz[3] - measurement[0,1]) < prep_params[2]:
            breaker = False
    elif np.square(prep_meanz[2] - measurement[1,0]) +\
         np.square(prep_meanz[3] - measurement[1,1]) < prep_params[2]:
            breaker = False
    elif np.square(prep_meanz[4] - measurement[0,0]) +\
         np.square(prep_meanz[5] - measurement[0,1]) < prep_params[2]:
            breaker = False
    elif np.square(prep_meanz[4] - measurement[1,0]) +\
         np.square(prep_meanz[5] - measurement[1,1]) < prep_params[2]:
            breaker = False
    elif has2 and np.square(prep_meanz[0] - measurement[2,0]) +\
                  np.square(prep_meanz[1] - measurement[2,1]) < prep_params[2]:
            breaker = False
    elif has2 and np.square(prep_meanz[2] - measurement[2,0]) +\
                  np.square(prep_meanz[3] - measurement[2,1]) < prep_params[2]:
            breaker = False
    elif has2 and np.square(prep_meanz[4] - measurement[2,0]) +\
                  np.square(prep_meanz[5] - measurement[2,1]) < prep_params[2]:
            breaker = False
    if breaker: return 0.
    
    msmt2 = np.zeros((8,))
    include = HHH(prep_params[0], prep_params[1], measurement, msmt2)
    deviation = msmt2[include] - prep_meanz[include]
    scov = prep_covz[include,:][:,include]
    s, u = np.linalg.eigh(scov)
    log_pdet = np.sum(np.log(s))
    lterm = np.dot(np.square(np.dot(deviation, u)), 1./s)
    return np.exp(-.5 * (_pdf_const*include.shape[0] + log_pdet + lterm))


@nb.jit(nb.void(nb.f8[:], nb.f8[:,:], nb.f8[:], nb.f8[:], nb.f8[:], nb.f8[:,:],
                nb.f8[:,:], nb.f8[:,:], nb.f8[:,:]), 
        nopython=True, parallel=parallel)
def update(sample, msmt, output, prep_params, prep_meanz, prep_gain, prep_covz,
           new_covs, prep_H):
    output[:] = sample
    new_mean = output[1:7]
    covAsSquare(output, new_covs)
    #new_covs = output[7:].reshape((6,6))
    # standard Kalman filter
    msmt2 = np.zeros((8,))
    include = HHH(prep_params[0], prep_params[1], msmt, msmt2)
    deviation = msmt2[include] - prep_meanz[include]
    #scov += np.outer(angcol*.01, angcol)
    kalman_gain = np.linalg.solve(prep_covz[include,:][:,include],
                                  prep_gain[:,include].T).T
#    kalman_gain = np.dot(prep[3][:,include],
#                         np.linalg.inv(prep[4][include,:][:,include]))
    new_mean += np.dot(kalman_gain, deviation)
    new_covs -= np.dot(kalman_gain, prep_gain[:,include].T)
    
    # fix symmetry errors
    if new_mean[2] < 0:
        new_covs[2,:] *= -1
        new_covs[:,2] *= -1
        new_mean[2] *= -1
    if new_mean[3] < 0:
        new_covs[3,:] *= -1
        new_covs[:,3] *= -1
        new_mean[3] *= -1
    
    covAsList(output, new_covs)
    
    # hardcode angle update
    c2 = msmt[1,0]-msmt[0,0]
    s2 = msmt[1,1]-msmt[0,1]
    d2 = np.hypot(c2, s2)
    c2 /= d2
    s2 /= d2
    c = np.cos(sample[0])
    s = np.sin(sample[0])
    if abs(c*c2 + s*s2) < abs(c*s2 - s*c2):
        c2,s2 = (s2,-c2)
    if c*c2 + s*s2 < 0:
        c2 = -c2
        s2 = -s2
    output[0] = np.arctan2(s + s2*angle_update_rate, c + c2*angle_update_rate)

    
def orderForPrune(samples): return samples[:,:5]

def report(samples): return samples[:,[1,2,0,3,4]]


def sampleFromMsmt(msmt):
    """ take msmt and reverse-engineer a sample """
    vector1 = (msmt[1,0]-msmt[0,0], msmt[1,1]-msmt[0,1])
    min_l = np.hypot(vector1[0], vector1[1])/2
    vector1 = (vector1[0]/min_l/2, vector1[1]/min_l/2)
    if msmt[2,2] >= 0: # two edges
        max_l = min_l if msmt[0,2] else newsample_max_dim
        vector2 = (msmt[1,0]-msmt[2,0], msmt[1,1]-msmt[2,1])
        min_w = np.hypot(vector2[0], vector2[1])/2
        vector2 = (vector2[0]/min_w/2, vector2[1]/min_w/2)
        # debug: assert perpendicular edges
        assert abs(vector1[0]*vector2[0]+vector1[1]*vector2[1]) < 1e-2
        max_w = min_w if msmt[2,2] else newsample_max_dim
    else: # one edge visible
        max_l = min_l if msmt[0,2] and msmt[1,2] else newsample_max_dim
        min_w = 0.01
        max_w = newsample_max_dim
        vector2 = (vector1[1], -vector1[0])
        # facing towards visible edge
        if vector2[0]*msmt[0,0] + vector2[1]*msmt[0,1] > 0:
            vector2 = (-vector2[0], -vector2[1])
    max_l = max(max_l, min_l)
    mean_l = min_l/2 + max_l/2
    max_w = max(max_w, min_w)
    mean_w = min_w/2 + max_w/2
    mean_x = msmt[1,0] - vector1[0]*mean_l - vector2[0]*mean_w
    mean_y = msmt[1,1] - vector1[1]*mean_l - vector2[1]*mean_w
    angle = np.arctan2(vector1[1], vector1[0])
    angle = angle % np.pi
    
    entry_sample = np.zeros((43,))
    entry_sample[[7,14]] = newsample_min_pos_var
    entry_sample[[21,28]] = newsample_min_dim_var
    entry_sample[[35,42]] = newsample_min_speed_var
    lvariance = np.square(max_l - min_l) / 16
    wvariance = np.square(max_w - min_w) / 16
    csq = np.square(np.cos(angle))
    ssq = np.square(np.sin(angle))
    entry_sample[7] += lvariance*csq + wvariance*ssq
    entry_sample[14] += lvariance*ssq + wvariance*csq
    entry_sample[21] += lvariance
    entry_sample[28] += wvariance
    entry_sample[:5] = (angle, mean_x, mean_y, mean_l, mean_w)
    
    return entry_sample



if __name__ == '__main__':
    print("testing the box tracker!")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    plt.ioff()
    fig, axs = plt.subplots(2,2, figsize=(8,8))
    
    sample1 = np.zeros((nfeatures,))
    sample1[0] = 0.
    sample1[1] = 5.
    sample1[3] = 1.
    sample1[4] = .5
    sample1[[7,14,21,28,35,42]] = 1.
    
    sample2 = np.zeros((nfeatures,))
    sample2[0] = np.pi/4.
    sample2[1] = 5.
    sample2[3] = 1.
    sample2[4] = .5
    sample2[[7,14,21,28,35,42]] = 1.
    
    ax = axs[0,0]
    #ax.title('sample means')
    ax.set_xlim(2.5, 6.5)
    ax.set_ylim(-2, 2)
    r = sample1
    c = np.cos(r[0])
    s = np.sin(r[0])
    ax.add_patch(mpatches.Rectangle((r[1]-r[3]*c+r[4]*s, r[2]-r[4]*c-r[3]*s),
                        r[3]*2, r[4]*2, r[0]*180/np.pi, fill=False, color='r'))
    r = sample2
    c = np.cos(r[0])
    s = np.sin(r[0])
    ax.add_patch(mpatches.Rectangle((r[1]-r[3]*c+r[4]*s, r[2]-r[4]*c-r[3]*s),
                        r[3]*2, r[4]*2, r[0]*180/np.pi, fill=False, color='g'))
    
        
    preps1 = preps
    prepObject(sample1, *preps1)
    preps2 = tuple(prep_term.copy() for prep_term in preps)
    prepObject(sample2, *preps2)
    
    ax = axs[0,1]
    #ax.title('expected msmt')
    ax.set_xlim(2.5, 6.5)
    ax.set_ylim(-2, 2)
    ax.plot(preps1[1][:6:2], preps1[1][1:6:2], 'ro-')
    ax.plot(preps2[1][:6:2], preps2[1][1:6:2], 'go-')
    
    
    # make measurement out of other rectangle
    x = 5.
    y = .1
    l = 1.
    w = .5
    angle = np.pi/6
    c = np.cos(angle)
    s = np.sin(angle)
    measurement = np.array([[x-c*l+s*w, y-s*l-c*w, 1.],
                            [x-c*l-s*w, y-s*l+c*w, 0.],
                            [x+c*l-s*w, y+s*l+c*w, 1.]])
    
    ax = axs[1,0]
    #ax.title('msmt')
    ax.set_xlim(2.5, 6.5)
    ax.set_ylim(-2, 2)
    ax.plot(measurement[:,0], measurement[:,1], 'bo-')
    
    
    l1 = likelihood(sample1, measurement, *preps1)
    updated1 = sample1.copy()
    update(sample1, measurement, updated1, *preps1)
    l2 = likelihood(sample2, measurement, *preps2)
    updated2 = np.zeros((nfeatures,))
    update(sample2, measurement, updated2, *preps2)
    
    ax = axs[1,1]
    ax.set_xlim(2.5, 6.5)
    ax.set_ylim(-2, 2)
    r = updated1
    c = np.cos(r[0])
    s = np.sin(r[0])
    ax.add_patch(mpatches.Rectangle((r[1]-r[3]*c+r[4]*s, r[2]-r[4]*c-r[3]*s),
                        r[3]*2, r[4]*2, r[0]*180/np.pi, fill=False, color='r'))
    r = updated2
    c = np.cos(r[0])
    s = np.sin(r[0])
    ax.add_patch(mpatches.Rectangle((r[1]-r[3]*c+r[4]*s, r[2]-r[4]*c-r[3]*s),
                        r[3]*2, r[4]*2, r[0]*180/np.pi, fill=False, color='g'))
    plt.show()