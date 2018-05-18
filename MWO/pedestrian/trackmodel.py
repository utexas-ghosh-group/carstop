#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 5/17/18
samples = mean x, mean x', mean w, var x, var x', var w, cov x/x', cov x/w, cov x'/w
          mean y, mean y', mean h, var y, var y', var h, cov y/y', cov y/h, cov y'/h
          detectability, occludability
horizontal and vertical features are independent
"""
import numpy as np

data_folder = 'train/MOT17-04-FRCNN/'
fps=30
skip=2

sample_nft = 20

# reload data to get params
# this is really lazy, but I don't want to put the whole code into a class
# or something
data = np.loadtxt(data_folder + 'det.txt', delimiter=',')
minx = np.min(data[:,1])
maxx = np.max(data[:,1]+data[:,3])
miny = np.min(data[:,2])
maxy = np.max(data[:,2]+data[:,4])
avgwidth = np.mean(data[:,3]) / 2.
avgheight = np.mean(data[:,4]) / 2.
stdwidth = np.std(data[:,3]) / 2.
stdheight = np.std(data[:,4]) / 2.
imshape_corr = np.corrcoef(data[:,3], data[:,4])[0][1]
stdmotion = avgheight/fps*skip # assume person moves 1-person-per-second
avg_num_msmts = float(data.shape[0]) / (data[-1,0] - data[0,0] + 1)
del data # to save memory


## initialization
initial_detectability = .9
initial_occludability = .9
initial_sample = np.array([(maxx+minx)/2., 0, avgwidth,
                           (maxx-minx)**2, stdmotion**2*4, stdwidth**2*16, 0, 0, 0,
                           (maxy+miny)/2., 0, avgheight,
                           (maxy-miny)**2, stdmotion**2*4, stdheight**2*16, 0, 0, 0,
                           initial_detectability, initial_occludability])
# only a fraction of this probability is actually in image zone...
initial_cardinality = avg_num_msmts / .16


## prediction
detectability_stationary = .9# .93#
detectability_half_second_ratio = .99
occludability_stationary = .8# .00001#
occludability_half_second_ratio = .8# 1.#
process_noise = np.array([stdmotion/2, stdmotion/4, stdmotion/3])**2

# markov transition probabilities for each step
detectable_maintain = 1 - (1 - detectability_stationary) * (1 - 
                                    (1-detectability_half_second_ratio)**(2./fps*skip))
undetect_maintain = 1 - (detectability_stationary) * (1 - 
                                    (1-detectability_half_second_ratio)**(2./fps*skip))
occludable_maintain = 1 - (1 - occludability_stationary) * (1 - 
                                    (1-occludability_half_second_ratio)**(2./fps*skip))
unocclude_maintain = 1 - (occludability_stationary) * (1 - 
                                    (1-occludability_half_second_ratio)**(2./fps*skip))
def predict(samples):
    samples[:,0] += samples[:,1]
    samples[:,3] += samples[:,6]
    samples[:,6] += samples[:,4]
    samples[:,3] += samples[:,6]
    samples[:,7] += samples[:,8]
    samples[:,3:6] += process_noise
    
    samples[:,9] += samples[:,10]
    samples[:,12] += samples[:,15]
    samples[:,15] += samples[:,13]
    samples[:,12] += samples[:,15]
    samples[:,16] += samples[:,17]
    samples[:,12:15] += process_noise
    
    samples[:,18] = 1 - undetect_maintain +\
                   (detectable_maintain+undetect_maintain-1)*samples[:,18]
    samples[:,19] = 1 - unocclude_maintain +\
                   (occludable_maintain+unocclude_maintain-1)*samples[:,19]
                   
                   
## survival
survival_out_of_zone = .4
survival_in_zone = 1 - .075 / fps * skip
def survival(samples):
    out_of_zone = (samples[:,0] < minx+stdmotion) |\
                  (samples[:,0] > maxx-stdmotion) |\
                  (samples[:,9] < miny+stdmotion) |\
                  (samples[:,9] > maxy-stdmotion) |\
                  (samples[:,2] < -1) | (samples[:,11] < -1)
    return np.where(out_of_zone, survival_out_of_zone, survival_in_zone)
                   

## entry
entry_cardinality = 10.
secret_entry_card = 1. / fps * skip
entry_detectability = .9
entry_occludability = .5

entry_sample_top = [(maxx+minx)/2., 0, avgwidth-stdwidth,
                    (maxx-minx)**2, stdmotion**2*4, stdwidth**2, 0, 0, 0,
                    miny, stdmotion/2, avgheight,
                    stdmotion**2, stdmotion**2, stdheight**2, 0, 0, 0,
                    entry_detectability, entry_occludability]
entry_sample_bottom = [(maxx+minx)/2., 0, avgwidth+stdwidth,
                       (maxx-minx)**2, stdmotion**2*4, stdwidth**2, 0, 0, 0,
                       maxy, -stdmotion/2, avgheight,
                       stdmotion**2, stdmotion**2, stdheight**2, 0, 0, 0,
                       entry_detectability, entry_occludability]
entry_sample_left = [minx, stdmotion/2, avgwidth,
                     stdmotion**2, stdmotion**2, stdwidth**2, 0, 0, 0,
                     (maxy+miny)/2., 0, avgheight,
                     (maxy-miny)**2, stdmotion**2*4, stdheight**2, 0, 0, 0,
                     entry_detectability, entry_occludability]
entry_sample_right = [maxx, -stdmotion/2, avgwidth,
                      stdmotion**2, stdmotion**2, stdwidth**2, 0, 0, 0,
                      (maxy+miny)/2., 0, avgheight,
                      (maxy-miny)**2, stdmotion**2*4, stdheight**2, 0, 0, 0,
                      entry_detectability, entry_occludability]
entry_samples = np.array([entry_sample_top, entry_sample_bottom, entry_sample_left,
                          entry_sample_right, initial_sample])
side_entry_cardinality = entry_cardinality / fps * skip / 4 / .36
entry_cardinality = [side_entry_cardinality] * 4 + [secret_entry_card]


## detection and occlusion
def detect(samples): return samples[:,18]

def prepMeasurements4Occlusion(measurements):
    return np.stack((measurements[:,1],
                     measurements[:,1] + measurements[:,3],
                     measurements[:,2],
                     measurements[:,2] + measurements[:,4]), axis=-1)
    
def prepObjects4Occlusion(samples):
    return np.stack((samples[:,0] - samples[:,2],
                     samples[:,0] + samples[:,2],
                     samples[:,9] - samples[:,11],
                     samples[:,9] + samples[:,11]), axis=-1)

bottom_gap_zero = -.1
bottom_gap_one = .25
def occlude(samples, obstacles, existences):
    total_visible = np.ones((samples.shape[0],))
    for k, obst in enumerate(obstacles):
        itx_area = intersectArea(samples[:,0], samples[:,2], samples[:,9], samples[:,11],
                                 obst[0], obst[1], obst[2], obst[3])
        bottom_diff = (obst[3] - samples[:,9] - samples[:,11])/(obst[3]-obst[2])
        block = np.minimum(1, np.maximum(0, itx_area*2))
        block *= np.minimum(1, np.maximum(0,
                 (bottom_diff-bottom_gap_zero)/(bottom_gap_one-bottom_gap_zero)))
        total_visible *= 1 - block * existences[k]
    return samples[:,19] * (1 - total_visible)

# make sure you don't let an object occlude itself
def occludeOWO2(samples, ids, included_ids, objects, object_exists):
    total_visible = np.ones((samples.shape[0],))
    for k in range(object_exists.shape[0]):
        included_samples = ids[:,included_ids[k]]==False
        obst = objects[k]
        itx_area = intersectArea(samples[:,0], samples[:,2], samples[:,9], samples[:,11],
                                 obst[0], obst[1], obst[2], obst[3])
        bottom_diff = (obst[3] - samples[:,9] - samples[:,11])/(obst[3]-obst[2])
        block = np.minimum(1, np.maximum(0, itx_area*2))
        block *= np.minimum(1, np.maximum(0,
                 (bottom_diff-bottom_gap_zero)/(bottom_gap_one-bottom_gap_zero)))
        total_visible[included_samples] *= 1 - block[included_samples] * object_exists[k]
    return samples[:,19] * (1 - total_visible)

# ( p(x) - p(x)P_D(x) ) / ( 1 - P_D )
# ( p(x) - p(x)P_D(x) + p(x)P_D(x)P_O(x) ) / ( 1 - P_D + P_OD )
# add output of miss() to input here for convenience
def updateOnMiss(samples, miss, measurements, out=None):
    if out is None:
        out = np.empty(samples.shape)
    out[:,:18] = samples[:,:18] # no kalman filtering
    out[:,18] = 1 - (1 - samples[:,18]) / miss
    out[:,19] = 1 - (1 - samples[:,18])*(1 - samples[:,19]) / miss
    return out

# a very rough approximation of posterior mean given miss...
# not used in results
#sft = 6
#def updateOnMiss(samples, miss, measurements, out=None):
#    if out is None:
#        out = samples.copy()
#    else:
#        out[:] = samples
#    # wonky jazz
#    trials = np.empty((out.shape[0], 3, 3))
#    for x_idx, x_off in enumerate([-sft, 0, sft]):
#        for y_idx, y_off in enumerate([-sft, 0, sft]):
#            out[:,0] = samples[:,0] + x_off
#            out[:,9] = samples[:,9] + y_off
#            trials[:, x_idx, y_idx] = occlude(out, measurements) * out[:,19]
#    xtrials = np.sum(trials, axis=2)
#    ytrials = np.sum(trials, axis=1)
#    total_trials = np.sum(xtrials, axis=1) + 1 - out[:,19]
#    out[:,0] = samples[:,0] + sft * (xtrials[:,2]-xtrials[:,0])/total_trials
#    out[:,9] = samples[:,9] + sft * (ytrials[:,2]-ytrials[:,0])/total_trials
#    
#    out[:,18] = 1 - (1 - samples[:,18]) / miss
#    out[:,19] = 1 - (1 - samples[:,18])*(1 - samples[:,19]) / miss
#    return out


## sensing
sensor_noise_horz = 12. ** 2
sensor_noise_vert = 12. ** 2   

# set up so that likelihood is exp(-.5 * (sum_i(ai xi) + b))
# where x is deviation
# a_i = terms in measurement precision matrix,
# b = log of normal pdf constant
def prepLikelihood(samples):
    piconst = np.log(2*np.pi)
    prep = np.empty((samples.shape[0], 16))
    
    varl = samples[:,3] + samples[:,5] - 2*samples[:,7] + sensor_noise_horz
    varr = samples[:,3] + samples[:,5] + 2*samples[:,7] + sensor_noise_horz
    covlr = samples[:,3] - samples[:,5]
    detlr = varl * varr - covlr*covlr
    prep[:,0] = 1./varl
    prep[:,1] = 1./varr
    prep[:,2] = varr / detlr
    prep[:,3] = varl / detlr
    prep[:,4] = -covlr / detlr
    prep[:,5] = np.log(varl) + piconst
    prep[:,6] = np.log(varr) + piconst
    prep[:,7] = np.log(detlr) + piconst*2
    
    varu = samples[:,12] + samples[:,14] - 2*samples[:,16] + sensor_noise_vert
    vard = samples[:,12] + samples[:,14] + 2*samples[:,16] + sensor_noise_vert
    covdu = samples[:,12] - samples[:,14]
    detdu = vard * varu - covdu*covdu
    prep[:,8] = 1./varu
    prep[:,9] = 1./vard
    prep[:,10] = vard / detdu
    prep[:,11] = varu / detdu
    prep[:,12] = -covdu / detdu
    prep[:,13] = np.log(varu) + piconst
    prep[:,14] = np.log(vard) + piconst
    prep[:,15] = np.log(detdu) + piconst*2
    return prep

def likelihood(samples, prep, measurement):
    msmt = preprocessMeasurement(measurement)
    if msmt[4] and msmt[5]: # both sides present
        devl = msmt[0] - samples[:,0] + samples[:,2]
        devr = msmt[1] - samples[:,0] - samples[:,2]
        expterm = prep[:,7] +\
                  prep[:,2] * devl * devl +\
                  prep[:,3] * devr * devr +\
                  prep[:,4] * devl * devr * 2
    elif msmt[4]: # only left side present
        devl = msmt[0] - samples[:,0] + samples[:,2]
        expterm = prep[:,5] + prep[:,0] * devl * devl
    elif msmt[5]:
        devr = msmt[1] - samples[:,0] - samples[:,2]
        expterm = prep[:,6] + prep[:,1] * devr * devr
    else:
        raise Exception
    
    if msmt[6] and msmt[7]:
        devl = msmt[2] - samples[:,9] + samples[:,11]
        devr = msmt[3] - samples[:,9] - samples[:,11]
        expterm += prep[:,15] +\
                   prep[:,10] * devl * devl +\
                   prep[:,11] * devr * devr +\
                   prep[:,12] * devl * devr * 2
    elif msmt[6]: # only top side present
        devl = msmt[2] - samples[:,9] + samples[:,11]
        expterm += prep[:,13] + prep[:,8] * devl * devl
    elif msmt[5]:
        devr = msmt[3] - samples[:,9] - samples[:,11]
        expterm += prep[:,14] + prep[:,9] * devr * devr
    else:
        raise Exception
        
    return np.exp(-.5 * expterm)


def prepUpdate(samples):
    return prepLikelihood(samples)
    
# samples = old samples
# out = new samples
def update(samples, prep, measurement, out=None):
    msmt = preprocessMeasurement(measurement)
    if out is None:
        out = samples.copy()
    else:
        out[:] = samples
        
    devl = msmt[0] - samples[:,0] + samples[:,2]
    devr = msmt[1] - samples[:,0] - samples[:,2]
    sxl = samples[:,3] - samples[:,7]
    svl = samples[:,6] - samples[:,8]
    swl = samples[:,7] - samples[:,5]
    sxr = samples[:,3] + samples[:,7]
    svr = samples[:,6] + samples[:,8]
    swr = samples[:,7] + samples[:,5]
    
    if not msmt[5]: # just left
        out[:,0] += sxl * prep[:,0] * devl
        out[:,1] += svl * prep[:,0] * devl
        out[:,2] += swl * prep[:,0] * devl
        out[:,3] -= sxl * sxl * prep[:,0]
        out[:,4] -= svl * svl * prep[:,0]
        out[:,5] -= swl * swl * prep[:,0]
        out[:,6] -= sxl * svl * prep[:,0]
        out[:,7] -= sxl * swl * prep[:,0]
        out[:,8] -= svl * swl * prep[:,0]
    elif not msmt[4]: # just right
        out[:,0] += sxr * prep[:,1] * devr
        out[:,1] += svr * prep[:,1] * devr
        out[:,2] += swr * prep[:,1] * devr
        out[:,3] -= sxr * sxr * prep[:,1]
        out[:,4] -= svr * svr * prep[:,1]
        out[:,5] -= swr * swr * prep[:,1]
        out[:,6] -= sxr * svr * prep[:,1]
        out[:,7] -= sxr * swr * prep[:,1]
        out[:,8] -= svr * swr * prep[:,1]
    else:
        PH = np.array(((sxl,svl,swl),(sxr,svr,swr))).T
        Kl = prep[:,None,2] * PH[:,:,0] + prep[:,None,4] * PH[:,:,1]
        Kr = prep[:,None,3] * PH[:,:,1] + prep[:,None,4] * PH[:,:,0]
        out[:,:3] += Kl * devl[:,None] + Kr * devr[:,None]
        out[:,3] -= Kl[:,0] * PH[:,0,0] + Kr[:,0] * PH[:,0,1]
        out[:,4] -= Kl[:,1] * PH[:,1,0] + Kr[:,1] * PH[:,1,1]
        out[:,5] -= Kl[:,2] * PH[:,2,0] + Kr[:,2] * PH[:,2,1]
        out[:,6] -= Kl[:,0] * PH[:,1,0] + Kr[:,0] * PH[:,1,1]
        out[:,7] -= Kl[:,0] * PH[:,2,0] + Kr[:,0] * PH[:,2,1]
        out[:,8] -= Kl[:,1] * PH[:,2,0] + Kr[:,1] * PH[:,2,1]
        
    assert np.all(determinant3(samples[:,3:9] - out[:,3:9]) >= -1e-1)
        
    devl = msmt[2] - samples[:,9] + samples[:,11]
    devr = msmt[3] - samples[:,9] - samples[:,11]
    sxl = samples[:,12] - samples[:,16]
    svl = samples[:,15] - samples[:,17]
    swl = samples[:,16] - samples[:,14]
    sxr = samples[:,12] + samples[:,16]
    svr = samples[:,15] + samples[:,17]
    swr = samples[:,16] + samples[:,14]
    
    if not msmt[7]: # just top
        out[:,9] += sxl * prep[:,8] * devl
        out[:,10] += svl * prep[:,8] * devl
        out[:,11] += swl * prep[:,8] * devl
        out[:,12] -= sxl * sxl * prep[:,8]
        out[:,13] -= svl * svl * prep[:,8]
        out[:,14] -= swl * swl * prep[:,8]
        out[:,15] -= sxl * svl * prep[:,8]
        out[:,16] -= sxl * swl * prep[:,8]
        out[:,17] -= svl * swl * prep[:,8]
    elif not msmt[6]: # just bottom
        out[:,9] += sxr * prep[:,9] * devr
        out[:,10] += svr * prep[:,9] * devr
        out[:,11] += swr * prep[:,9] * devr
        out[:,12] -= sxr * sxr * prep[:,9]
        out[:,13] -= svr * svr * prep[:,9]
        out[:,14] -= swr * swr * prep[:,9]
        out[:,15] -= sxr * svr * prep[:,9]
        out[:,16] -= sxr * swr * prep[:,9]
        out[:,17] -= svr * swr * prep[:,9]
    else:
        PH = np.array(((sxl,svl,swl),(sxr,svr,swr))).T
        Kl = prep[:,None,10] * PH[:,:,0] + prep[:,None,12] * PH[:,:,1]
        Kr = prep[:,None,11] * PH[:,:,1] + prep[:,None,12] * PH[:,:,0]
        out[:,9:12] += Kl * devl[:,None] + Kr * devr[:,None]
        out[:,12] -= Kl[:,0] * PH[:,0,0] + Kr[:,0] * PH[:,0,1]
        out[:,13] -= Kl[:,1] * PH[:,1,0] + Kr[:,1] * PH[:,1,1]
        out[:,14] -= Kl[:,2] * PH[:,2,0] + Kr[:,2] * PH[:,2,1]
        out[:,15] -= Kl[:,0] * PH[:,1,0] + Kr[:,0] * PH[:,1,1]
        out[:,16] -= Kl[:,0] * PH[:,2,0] + Kr[:,0] * PH[:,2,1]
        out[:,17] -= Kl[:,1] * PH[:,2,0] + Kr[:,1] * PH[:,2,1]
                     
    assert np.all(determinant3(samples[:,12:18] - out[:,12:18]) >= -1e-1)
        
    out[:,18] = 1.
    return out
      
# slower but definitely correct
def updateDebug(samples, measurement):
    msmt = preprocessMeasurement(measurement)
    out = samples.copy()
    
    H = np.array([[1,0,-1],[1,0,1]])
    R = np.eye(2) * sensor_noise_horz
    mm = msmt[:2]
    if not msmt[5]:
        H = H[:1]
        R = R[:1,:1]
        mm = mm[:1]
    elif not msmt[4]:
        H = H[1:]
        R = R[1:,1:]
        mm = mm[1:]
    dev = mm - np.einsum(H, [1,2], samples[:,:3], [0,2], [0,1])
    P = np.array(((samples[:,3],samples[:,6],samples[:,7]),
                  (samples[:,6],samples[:,4],samples[:,8]),
                  (samples[:,7],samples[:,8],samples[:,5]))).T
    PH = np.einsum(P, [0,1,2], H, [3,2], [0,1,3])
    prec = np.linalg.inv(np.einsum(PH,[0,1,2], H,[3,1], [0,3,2]) + R)
    K = np.einsum(PH,[0,1,2],prec,[0,2,3], [0,1,3])
    out[:,:3] += np.einsum(K, [0,1,2], dev, [0,2], [0,1])
    change = np.einsum(K, [0,1,2], PH, [0,3,2], [0,1,3])
    out[:,3] -= change[:,0,0]
    out[:,4] -= change[:,1,1]
    out[:,5] -= change[:,2,2]
    out[:,6] -= change[:,0,1]
    out[:,7] -= change[:,0,2]
    out[:,8] -= change[:,1,2]
    
    H = np.array([[1,0,-1],[1,0,1]])
    R = np.eye(2) * sensor_noise_vert
    mm = msmt[2:4]
    if not msmt[7]:
        H = H[:1]
        R = R[:1,:1]
        mm = mm[:1]
    elif not msmt[6]:
        H = H[1:]
        R = R[1:,1:]
        mm = mm[1:]
    dev = mm - np.einsum(H, [1,2], samples[:,9:12], [0,2], [0,1])
    P = np.array(((samples[:,12],samples[:,15],samples[:,16]),
                  (samples[:,15],samples[:,13],samples[:,17]),
                  (samples[:,16],samples[:,17],samples[:,14]))).T
    PH = np.einsum(P, [0,1,2], H, [3,2], [0,1,3])
    prec = np.linalg.inv(np.einsum(PH,[0,1,2],H,[3,1],[0,3,2]) + R)
    K = np.einsum(PH,[0,1,2],prec,[0,2,3], [0,1,3])
    out[:,9:12] += np.einsum(K, [0,1,2], dev, [0,2], [0,1])
    change = np.einsum(K, [0,1,2], PH, [0,3,2], [0,1,3])
    out[:,12] -= change[:,0,0]
    out[:,13] -= change[:,1,1]
    out[:,14] -= change[:,2,2]
    out[:,15] -= change[:,0,1]
    out[:,16] -= change[:,0,2]
    out[:,17] -= change[:,1,2]
    
    out[:,18] = 1.
    return out
    
  

## output
def output(samples):
    return np.stack((samples[:,0] - samples[:,2],
                     samples[:,9] - samples[:,11],
                     samples[:,2]*2, samples[:,11]*2), axis=-1)

def debug(samples):
    assert np.all(samples[:,18] > -1e-10)
    assert np.all(samples[:,18] < 1+1e-10)
    assert np.all(samples[:,19] > -1e-10)
    assert np.all(samples[:,19] < 1+1e-10)
    assert np.all(samples[:,0] > minx - (maxx-minx)/3)
    assert np.all(samples[:,0] < maxx + (maxx-minx)/3)
    assert np.all(samples[:,9] > miny - (maxy-miny)/3)
    assert np.all(samples[:,9] < maxy + (maxy-miny)/3)
    assert np.all(samples[:,2] > -1)
    assert np.all(samples[:,11] > -1)
    assert np.all(samples[:,3:6] > 1e-2)
    assert np.all(samples[:,12:15] > 1e-2)
    assert np.all(determinant3(samples[:,3:9]) > -1e-4)
    assert np.all(determinant3(samples[:,12:18]) > -1e-4)


## helpers
def determinant3(sigma):
    return sigma[:,0]*sigma[:,1]*sigma[:,2] -\
           sigma[:,0]*sigma[:,5]*sigma[:,5] -\
           sigma[:,1]*sigma[:,4]*sigma[:,4] -\
           sigma[:,2]*sigma[:,3]*sigma[:,3] +\
           2*sigma[:,3]*sigma[:,4]*sigma[:,5]    

# in: time, left, top, width, height
# out: left, right, top, bottom, left is valid, right is valid, ...
def preprocessMeasurement(msmt):
    msmt = msmt[1:5].copy()
    msmt2 = np.zeros((8,))
    if msmt[0] > minx:
        msmt2[4] = 1
        msmt2[0] = msmt[0]
    if msmt[0]+msmt[2] < maxx:
        msmt2[5] = 1
        msmt2[1] = msmt[0]+msmt[2]
    if msmt[1] > miny:
        msmt2[6] = 1
        msmt2[2] = msmt[1]
    if msmt[1]+msmt[3] < maxy:
        msmt2[7] = 1
        msmt2[3] = msmt[1]+msmt[3]
    return msmt2

def intersectArea(x, width, y, height, bleft, bright, bup, bdown):
    itx_width = np.maximum(0, 
                    np.minimum(x + width, bright) -
                    np.maximum(x - width, bleft))
    itx_height = np.maximum(0, 
                    np.minimum(y + height, bdown) -
                    np.maximum(y - height, bup))
    return itx_width * itx_height / width / height / 4