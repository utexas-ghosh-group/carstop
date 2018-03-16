#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 3/15/18
[x,y,height,vx,vy,detect_prob]
measurements = center of x,y (width estimate added), height
not width because width & height very highly correlated
H = [I 0]
F = [1 0 0 1 0
     0 1 0 0 1
     0 0 1 0 0 # maybe add 1/2 here or something
     0 0 0 1 0
     0 0 0 0 1]
"""

data_folder = 'train/MOT17-04-FRCNN/'
fps=30
skip=2
nsamples = 2048
nobjects = 72
detectability_stationary = .9#.93#
detectability_half_second_ratio = .99999#.99#
detectable_maintain = 1 - (1 - detectability_stationary) * (1 - 
                                    (1-detectability_half_second_ratio)**(2./fps*skip))
undetect_maintain = 1 - (detectability_stationary) * (1 - 
                                    (1-detectability_half_second_ratio)**(2./fps*skip))
occludability_stationary = .00001# .8#
occludability_half_second_ratio = 1.# .8#
occludable_maintain = 1 - (1 - occludability_stationary) * (1 - 
                                    (1-occludability_half_second_ratio)**(2./fps*skip))
unocclude_maintain = 1 - (occludability_stationary) * (1 - 
                                    (1-occludability_half_second_ratio)**(2./fps*skip))
initial_detectability = .9
initial_occludability = .9
entry_cardinality = 10.
n_entry_samples_per_side = 64
entry_detectability = .9
entry_occludability = .5
show_images = False
display_region = [300,200,540,900]#[0,200,300,900] # if None, display whole image, else (t,l,b,r)
make_video = True
video_name = 'track_test.mkv'
make_score = True
save_detections = True
detection_file_name = 'detect_train4N.txt'

import numpy as np
import scipy.optimize as spopt
from scipy.misc import imread
from scipy.spatial import cKDTree
from skvideo.io import FFmpegWriter as vwriter
import matplotlib.pyplot as plt
#from scipy.stats import truncnorm

important_count = 0

def getImg(frame, ext='.png'):
    return imread(data_folder + 'img1/{:06d}'.format(frame+1) + ext)

def drawBox2D(hmin,wmin,hmax,wmax, image, color=[40,40,150], linewidth=1,
              sides_to_include=[True,True,True,True]):
    lw = linewidth
    hmin = max(lw, int(hmin))
    hmax = min(image.shape[0] - lw, int(hmax))
    wmin = max(lw, int(wmin))
    wmax = min(image.shape[1] - lw, int(wmax))
    if sides_to_include[0]: # top
        image[hmin-lw:hmin+lw+1, wmin-lw:wmax+lw+1] = color
    if sides_to_include[1]: # left
        image[hmin-lw:hmax+lw+1, wmin-lw:wmin+lw+1] = color
    if sides_to_include[2]: # bottom
        image[hmax-lw:hmax+lw+1, wmin-lw:wmax+lw+1] = color
    if sides_to_include[3]: # right
        image[hmin-lw:hmax+lw+1, wmax-lw:wmax+lw+1] = color

def plotImg(img, region):
    if not region is None:
        img = img[region[0]:region[2], region[1]:region[3], :]
    fig = plt.figure(figsize=(img.shape[1]/96.,img.shape[0]/96.), dpi=96.)
    plt.axis('off')
    fig.gca().set_axis_off()
    plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)
    plt.margins(0,0)
    plt.imshow(img, interpolation='none')
    plt.show()


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


#_pdf_const = np.log(2*np.pi)*3
#def normalPdf(msmt, cov):
#    # from https://github.com/scipy/scipy/blob/v0.14.0/scipy/stats/_multivariate.py
#    s, u = np.linalg.eigh(cov)
#    U = u * np.sqrt(s)[:,None,:]
#    log_pdet = np.sum(np.log(s), axis=1)
#    lterm = np.sum(np.square(np.einsum(msmt, [0,1], U, [0,1,2],[0,2])), axis=1)
#    return np.exp(-.5 * (_pdf_const - log_pdet + lterm))

# H = [I 0]
def prepareFilterIO(covs, noise):
    nf = noise.shape[-1]
    z_prec = np.linalg.inv(covs[:,:nf,:nf] + noise)
    kalman_gain = np.einsum(covs[:,:,:nf], [0,1,2], z_prec, [0,2,3], [0,1,3])
    cov_dev = -np.einsum(kalman_gain, [0,1,2], covs[:,:nf,:], [0,2,3], [0,1,3])
    assert cov_dev.shape == covs.shape
    return z_prec, kalman_gain, cov_dev

def prepLikelihood(covs, noise):
    nf = noise.shape[-1]
    s, u = np.linalg.eigh(covs[:,:nf,:nf] + noise)
    U = np.transpose(u, (0,2,1)) / np.sqrt(s)[:,:,None]
    const = np.prod(s,axis=1)**-.5 / (2*np.pi)**(nf/2.)
    return const, U
def getLikelihood(dev, transform):
    return np.exp(-.5 * np.sum(np.square(np.einsum(transform, [0,1,2],
                                                   dev, [0,2], [0,1])), axis=1))


def DA_JAM(object_msmt, object_miss, miss_msmt):
    m = object_msmt.copy()
    for j in range(1000):
        rowsum = np.sum(m,axis=1) + object_miss
        m = object_msmt / (rowsum[:,None] - m)
        colsum = np.sum(m,axis=0) + miss_msmt
        m = object_msmt / (colsum - m)
    rowsum = np.sum(m,axis=1) + object_miss
    return m/rowsum[:,None], object_miss/rowsum, 1 - np.sum(m/rowsum[:,None],axis=0)


    
if __name__ == '__main__':
    data = np.loadtxt(data_folder + 'det.txt', delimiter=',')
    #data = data[:,[0,2,3,4,5,6]]
    #data[:,1] += data[:,3]/2. # center of x dim
    #data[:,2] += data[:,4]/2. # center of y
    min_confidence = np.min(data[:,5])
    times = list(np.where(np.diff(data[:,0])!=0)[0]+1)
    times = np.array([0] + times + [data.shape[0]], dtype=int)
        
    maxtime = times.shape[0]-1 #684#


    ## gather information from the data
    minx = np.min(data[:,1])
    maxx = np.max(data[:,1]+data[:,3])
    miny = np.min(data[:,2])
    maxy = np.max(data[:,2]+data[:,4])
    avgwidth = np.mean(data[:,3])
    avgheight = np.mean(data[:,4])
    stdwidth = np.std(data[:,3])
    stdheight = np.std(data[:,4])
    imshape_corr = np.corrcoef(data[:,3], data[:,4])[0][1]
    stdmotion = avgheight/fps*skip*.5 # assume person moves 1-person-per-second
    avg_num_msmts = float(times[-1]) / times.shape[0]
    
    # use this information to set hyperparameters
    min_vals = [minx, miny, avgwidth-stdwidth*2, avgheight-stdheight*2,
                -stdmotion*2, -stdmotion*2]
    max_vals = [maxx, maxy, avgwidth+stdwidth*2, avgheight+stdheight*2,
                stdmotion*2, stdmotion*2]
    initial_cardinality = avg_num_msmts * 1.5
    
    initial_cov = np.diag([(maxx-minx)/nsamples**.5, (maxy-miny)/nsamples**.5,
                           stdwidth, stdheight, stdmotion, stdmotion])**2
    initial_cov[2,3] = (initial_cov[2,2]*initial_cov[3,3])**.5 * imshape_corr
    initial_cov[3,2] = (initial_cov[2,2]*initial_cov[3,3])**.5 * imshape_corr
    Q = np.diag([stdmotion/2, stdmotion/2, stdmotion/3, stdmotion/3,
                 stdmotion/4, stdmotion/4]) ** 2
    Q[2,3] = (Q[2,2]*Q[3,3])**.5 * imshape_corr
    Q[3,2] = (Q[2,2]*Q[3,3])**.5 * imshape_corr
#    sensor_noise = np.diag([10.,10,15,15])**2
#    sensor_noise_vertical = np.diag([10.,40,15,60])**2 # on edge
#    sensor_noise_horizontal = np.diag([20.,10,30,15])**2 # on edge
    sensor_noise = np.diag([9.,9,12,12])**2
    sensor_noise_vertical = np.diag([8.,40,12,60])**2 # on edge
    sensor_noise_horizontal = np.diag([20.,8,30,12])**2 # on edge
    sensor_noise_both = np.diag([20.,40,30,60])**2
    entry_cov = np.diag([stdmotion, stdmotion, stdwidth, stdheight,
                         stdmotion, stdmotion])**2
    
    
    entry_means_top = np.tile([[0,maxy,avgwidth,avgheight,0,-stdmotion/2]],
                              (n_entry_samples_per_side,1))
    entry_means_top[:,0] = np.linspace(minx, maxx, n_entry_samples_per_side)
    entry_covs_top = np.tile(entry_cov, (n_entry_samples_per_side,1,1))
    entry_covs_top[:,0,0] = (maxx-minx)/n_entry_samples_per_side
    entry_means_bottom = np.tile([[0,miny,avgwidth,avgheight,0,stdmotion/2]],
                                 (n_entry_samples_per_side,1))
    entry_means_bottom[:,0] = np.linspace(minx, maxx, n_entry_samples_per_side)
    entry_covs_bottom = np.tile(entry_cov, (n_entry_samples_per_side,1,1))
    entry_covs_bottom[:,0,0] = (maxx-minx)/n_entry_samples_per_side
    entry_means_left = np.tile([[minx,0,avgwidth,avgheight,stdmotion/2,0]],
                               (n_entry_samples_per_side,1))
    entry_means_left[:,1] = np.linspace(miny, maxy, n_entry_samples_per_side)
    entry_covs_left = np.tile(entry_cov, (n_entry_samples_per_side,1,1))
    entry_covs_left[:,1,1] = (maxy-miny)/n_entry_samples_per_side
    entry_means_right = np.tile([[maxx,0,avgwidth,avgheight,-stdmotion/2,0]],
                                (n_entry_samples_per_side,1))
    entry_means_right[:,1] = np.linspace(miny, maxy, n_entry_samples_per_side)
    entry_covs_right = np.tile(entry_cov, (n_entry_samples_per_side,1,1))
    entry_covs_right[:,1,1] = (maxy-miny)/n_entry_samples_per_side
    entry_means = np.concatenate((entry_means_top, entry_means_bottom,
                                  entry_means_left, entry_means_right), axis=0)
    entry_covs = np.concatenate((entry_covs_top, entry_covs_bottom,
                                 entry_covs_left, entry_covs_right), axis=0)
    entry_exist = entry_cardinality / fps * skip / 4 / n_entry_samples_per_side
    
    # entry from anywhere - very low probability
    secret_entry_means = np.array([(maxx+minx)/2.,(maxy+miny)/2., avgwidth,
                                   avgheight, 0., 0.])
    secret_entry_covs = np.diag([(maxx-minx), (maxy-miny), stdwidth*4,
                                stdheight*4, stdmotion*2, stdmotion*2])**2
    secret_entry_exist = 1. / fps * skip
    

    # p(occluded|z,zv) = exp(-||z-zv||^2 / 2 / variance) * weight
    # halflife = (2 * variance * ln2)**.5
    occlusion_std = np.array([avgwidth / 2, avgheight / 2]) * (2 * np.log(2))**-.5
    occlude_noise = sensor_noise[:2,:2] + np.diag(occlusion_std ** 2)


    if make_video:
        video_out = vwriter(video_name, inputdict={'-r':'5'}, outputdict={'-an':'-y'})
    if make_score:
        truth = np.loadtxt(data_folder + 'gt.txt', delimiter=',')
        truth = truth[:,:6]
        truth_times = list(np.where(np.diff(truth[:,0])>0)[0]+1)
        truth_times = np.array([0] + truth_times + [truth.shape[0]], dtype=int)
        scores = []
    if save_detections:
        report_list = []
    
    
    ## initialize
    # two structures - will switch between these
    means = np.random.uniform(min_vals, max_vals, (nsamples,6))
    samples = np.zeros((nsamples,2)) + [initial_detectability, initial_occludability]
    covs = np.tile(initial_cov, (nsamples, 1, 1))
    existences = np.zeros((nsamples,)) + initial_cardinality/nsamples
    ids = np.zeros((nsamples, nobjects), dtype=bool)
    tracker_structure = (samples, means, covs, existences, ids)
    
    new_means = np.empty(means.shape)
    new_covs = np.empty(covs.shape)
    new_samples = np.empty(samples.shape)
    new_exist = np.empty(existences.shape)
    new_ids = np.empty(ids.shape, dtype=bool)
    new_tracker_structure = (new_samples, new_means, new_covs, new_exist, new_ids)
    
    unique_ids = np.zeros((nobjects,), dtype=int)
    unique_id_count = 0
        
    for time in range(0,maxtime,skip):
        measurements = data[times[time]:times[time+1]]
        
        ## predict
        means[:,:2] += means[:,4:]
        covs[:,:2,:] += covs[:,4:,:]
        covs[:,:,:2] += covs[:,:,4:]
        covs += Q
        samples[:,0] = 1 - undetect_maintain +\
                    (detectable_maintain+undetect_maintain-1)*samples[:,0]
        samples[:,1] = 1 - unocclude_maintain +\
                    (occludable_maintain+unocclude_maintain-1)*samples[:,1]
        out_of_zone = (means[:,0] < minx+stdmotion) |\
                      (means[:,0] > maxx-stdmotion) |\
                      (means[:,1] < miny+stdmotion) |\
                      (means[:,1] > maxy-stdmotion)
        existences *= np.where(out_of_zone, .4, 1 - .075/fps*skip)
        
        # entry
        if time % 2 == 0:
            idxs_to_remove = np.where(existences < entry_exist)[0]
            if idxs_to_remove.shape[0] > 0:
                idxs_to_add = np.arange(entry_means.shape[0],dtype=int)
                if idxs_to_remove.shape[0] > idxs_to_add.shape[0]:
                    idxs_to_remove = np.random.choice(idxs_to_remove,
                                                idxs_to_add.shape[0], replace=False)
                else:
                    idxs_to_add = np.random.choice(idxs_to_add,
                                            idxs_to_remove.shape[0], replace=False)
                means[idxs_to_remove] = entry_means[idxs_to_add]
                covs[idxs_to_remove] = entry_covs[idxs_to_add]
                samples[idxs_to_remove] = [entry_detectability, entry_occludability]
                ids[idxs_to_remove] = False
                existences[idxs_to_remove] = entry_exist
            # secret entry
            idx_to_remove = np.argmin(existences)
            if existences[idx_to_remove] < secret_entry_exist:
                means[idx_to_remove] = secret_entry_means
                covs[idx_to_remove] = secret_entry_covs
                samples[idx_to_remove] = [entry_detectability, .25]
                ids[idx_to_remove] = False
                existences[idx_to_remove] = entry_exist
            object_exist = np.einsum(existences, [0], ids, [0,1], [1])
            assert np.all(object_exist <= 1)
            
        ## update
        n_measurements = len(measurements)
        #fp_msmt = np.zeros((measurements.shape[0],)) + 3e-5
        #fp_msmt[measurements[:,5]>1] *= measurements[measurements[:,5]>1,5]**-2
        fp_msmt = 2e-10 / measurements[:,5]**2
        
        # parallel likelihood determination
        #measure_ll_const, measure_ll_transform = prepLikelihood(covs, sensor_noise)
        #occlude_ll_const, occlude_ll_transform = prepLikelihood(covs, occlude_noise)
        sample_DE = existences * samples[:,0]
        sample_msmt = np.ones((nsamples, n_measurements)) *\
                                sample_DE[:,None]# * measure_ll_const[:,None]
        sample_block = np.ones((nsamples, n_measurements))# * occlusion_weight
        for msmt_idx, measurement in enumerate(measurements):
            msmt = measurement[1:5].copy()
            horz_edge = False
            vert_edge = False
            if msmt[0]==minx:
                if msmt[2] < avgwidth:
                    msmt[0] += msmt[2] - avgwidth/2.
                    msmt[2] = avgwidth
                else:
                    msmt[0] += msmt[2]/2.
                horz_edge = True
            elif msmt[0]+msmt[2]==maxx:
                if msmt[2] < avgwidth:
                    msmt[0] += avgwidth/2.
                    msmt[2] = avgwidth
                else:
                    msmt[0] += msmt[2]/2.
                horz_edge = True
            else:
                msmt[0] += msmt[2]/2.
            if msmt[1]==miny:
                if msmt[3] < avgheight:
                    msmt[1] += msmt[3] - avgheight/2.
                    msmt[3] = avgheight
                else:
                    msmt[1] += msmt[3]/2.
                vert_edge=True
            elif msmt[1]+msmt[3]==maxy:
                if msmt[3] < avgheight:
                    msmt[1] += avgheight/2.
                    msmt[3] = avgheight
                else:
                    msmt[1] += msmt[3]/2.
                vert_edge=True
            else:
                msmt[1] += msmt[3]/2.
            if horz_edge and vert_edge:
                this_sensor_noise = sensor_noise_both
            elif horz_edge:
                this_sensor_noise = sensor_noise_horizontal
            elif vert_edge:
                this_sensor_noise = sensor_noise_vertical
            else:
                this_sensor_noise = sensor_noise
            deviation = msmt - means[:,:4]
            measure_ll_const, measure_ll_transform = prepLikelihood(covs, this_sensor_noise)
            sample_msmt[:,msmt_idx] *= getLikelihood(deviation, measure_ll_transform) * measure_ll_const
            #sample_block[:,msmt_idx] *= getLikelihood(deviation[:,:2], occlude_ll_transform)
            itx_width = np.maximum(0, 
                            np.minimum(means[:,0]+means[:,2]/2, msmt[0]+msmt[2]/2) -
                            np.maximum(means[:,0]-means[:,2]/2, msmt[0]-msmt[2]/2))
            itx_height = np.maximum(0, 
                            np.minimum(means[:,1]+means[:,3]/2, msmt[1]+msmt[3]/2) -
                            np.maximum(means[:,1]-means[:,3]/2, msmt[1]-msmt[3]/2))
            itx_area = itx_width * itx_height / np.prod(means[:,2:4],axis=1)
            bottom_diff = (msmt[1]+msmt[3]/2 - means[:,1] - means[:,3]/2)/msmt[3]
            sample_block[:,msmt_idx] *= np.minimum(np.maximum(itx_area*1.5, 0), 1)
            sample_block[:,msmt_idx] *= np.minimum(np.maximum(bottom_diff*2.5+.2,0),1)
        sample_visible = np.prod(1-sample_block, axis=1)
        sample_OV = 1 - samples[:,1] * (1 - sample_visible) # not (occludable & occluded)
        sample_DOV = samples[:,0] * sample_OV
        sample_DEV = sample_DE * sample_OV
        sample_miss = existences - sample_DEV # sample_DE # depends on kalman-ing
        #sample_block /= 1 - sample_block
        #sample_block *= sample_visible[:,None]
        
        # reduce to object likelihoods
        object_msmt = np.einsum(sample_msmt, [0,1], ids, [0,2], [2,1])
        #object_exist = np.einsum(existences, [0], ids, [0,2], [2])
        object_miss = 1 - np.einsum(sample_DEV, [0], ids, [0,2], [2])
        assert np.all(object_miss >= 0)
        assert np.all(object_miss <= 1)
        #assert np.all(object_exist < 1.02)
        #object_exist = np.minimum(object_exist, 1-1e-10)
        sprouts = np.any(ids,axis=1)==False
        miss_msmt = fp_msmt + np.sum(sample_msmt[sprouts],axis=0)
        
        # data association
        jam_object_msmt, jam_object_miss, jam_miss_msmt = DA_JAM(object_msmt,
                                                             object_miss, miss_msmt)
        assert np.all(jam_object_msmt < 1+1e-10)
        assert np.all(jam_object_miss < 1+1e-10)
        assert np.all(jam_miss_msmt < 1+1e-10)
        
        # expand to sample assignment probabilities
        match_object_msmt = jam_object_msmt.copy()
        match_object_msmt[object_msmt > 1e-30] /= object_msmt[object_msmt > 1e-30]
        match_object_miss = jam_object_miss.copy()
        match_object_miss[object_miss > 1e-30] /= object_miss[object_miss > 1e-30]
        match_miss_msmt = jam_miss_msmt.copy()
        match_miss_msmt[miss_msmt > 1e-30] /= miss_msmt[miss_msmt > 1e-30]
        match_sample_msmt = np.concatenate((sample_miss[:,None],
                                            sample_msmt)#,
                                            #sample_block), # depends on kalmaning
                                            , axis=1)
        for obj in range(nobjects):
            match_sample_msmt[ids[:,obj],1:n_measurements+1] *= match_object_msmt[obj,:]
            match_sample_msmt[ids[:,obj],0] *= match_object_miss[obj]
            #match_sample_msmt[ids[:,obj],n_measurements+1:] *= match_object_miss[obj,None]
        match_sample_msmt[sprouts,1:n_measurements+1] *= match_miss_msmt
        ##match_sample_msmt[sprouts,n_measurements+1:]
        assert np.all(match_sample_msmt < 1+1e-10)
        assert np.all(match_sample_msmt > -1e-10)
        # find the top entries in match_sample_msmt
        percentile = 100.*n_measurements/(n_measurements + 1)
        #percentile = 100.*2*n_measurements/(2*n_measurements+1) # depends on kalmaning
        min_value = np.percentile(match_sample_msmt, percentile, interpolation='lower')
        included_match_mtx = match_sample_msmt > min_value
        remaining_samples = nsamples - np.sum(match_sample_msmt > min_value)
        identicals_samp, identicals_msmt = np.where(match_sample_msmt == min_value)
        included_match_mtx[identicals_samp[:remaining_samples],
                           identicals_msmt[:remaining_samples]] = True
        match_idxs = np.cumsum(np.sum(included_match_mtx, axis=0))
        assert match_idxs[-1] == nsamples
        match_sample_msmt *= included_match_mtx
        # find id spaces for potential new objects
        # and handle unique labelling
        object_matches = np.einsum(match_sample_msmt,[0,1], ids,[0,2], [2])
        objects_ordered = np.argsort(object_matches)
        object_cum = np.cumsum(object_matches[objects_ordered])
        object_cum = np.append([0], object_cum[:n_measurements])
        new_msmt_matches = np.sum(match_sample_msmt[sprouts, 1:], axis=0)
        new_msmts_ordered = np.argsort(new_msmt_matches)
        new_msmt_cum = np.cumsum(new_msmt_matches[new_msmts_ordered])
        new_msmt_cum = np.append([0], new_msmt_cum)[::-1]
        n_new_ids = np.argmin(object_cum + new_msmt_cum)
        #new_objs = np.sum(match_sample_msmt[sprouts,1:], axis=0)
        replaced_ids = objects_ordered[:n_new_ids]
        replacing_ids = {msmt_idx:-1 for msmt_idx in range(n_measurements)}
        for new_id_num in range(n_new_ids):
            replacing_ids[new_msmts_ordered[-new_id_num-1]] = replaced_ids[new_id_num]
        ids[:,replaced_ids] = False
        unique_ids[replaced_ids] = range(n_new_ids)
        unique_ids[replaced_ids] += unique_id_count
        unique_id_count += n_new_ids
        
        # parallel (per measurement) update
        filter_normal = prepareFilterIO(covs, sensor_noise)[1:]
        filter_horz = prepareFilterIO(covs, sensor_noise_horizontal)[1:]
        filter_vert = prepareFilterIO(covs, sensor_noise_vertical)[1:]
        filter_both = prepareFilterIO(covs, sensor_noise_both)[1:]
#        occlude_prec, occlude_kg, occlude_dev = prepareFilterIO(covs, occlude_noise)
        include_nomatch = included_match_mtx[:,0]
        new_means[:match_idxs[0]] = means[include_nomatch]
        new_covs[:match_idxs[0]] = covs[include_nomatch]
        # calculations are joint probability of no measurement over total
        new_samples[:match_idxs[0], 0] = samples[include_nomatch,0] -\
                                            sample_DOV[include_nomatch]
        new_samples[:match_idxs[0], 1] = samples[include_nomatch,1] *\
                    (1-samples[include_nomatch,0]*sample_visible[include_nomatch])
        new_samples[:match_idxs[0], :] /= (1 - sample_DOV[include_nomatch])[:,None]
        new_exist[:match_idxs[0]] = match_sample_msmt[include_nomatch,0]
        new_ids[:match_idxs[0]] = ids[include_nomatch]
        for msmt_idx, measurement in enumerate(measurements):
            msmt = measurement[1:5].copy()
            horz_edge = False
            vert_edge = False
            if msmt[0]==minx:
                if msmt[2] < avgwidth:
                    msmt[0] += msmt[2] - avgwidth/2.
                    msmt[2] = avgwidth
                else:
                    msmt[0] += msmt[2]/2.
                horz_edge = True
            elif msmt[0]+msmt[2]==maxx:
                if msmt[2] < avgwidth:
                    msmt[0] += avgwidth/2.
                    msmt[2] = avgwidth
                else:
                    msmt[0] += msmt[2]/2.
                horz_edge = True
            else:
                msmt[0] += msmt[2]/2.
            if msmt[1]==miny:
                if msmt[3] < avgheight:
                    msmt[1] += msmt[3] - avgheight/2.
                    msmt[3] = avgheight
                else:
                    msmt[1] += msmt[3]/2.
                vert_edge=True
            elif msmt[1]+msmt[3]==maxy:
                if msmt[3] < avgheight:
                    msmt[1] += avgheight/2.
                    msmt[3] = avgheight
                else:
                    msmt[1] += msmt[3]/2.
                vert_edge=True
            else:
                msmt[1] += msmt[3]/2.
            if horz_edge and vert_edge:
                kalman_gain, cov_dev = filter_both
            elif horz_edge:
                kalman_gain, cov_dev = filter_horz
            elif vert_edge:
                kalman_gain, cov_dev = filter_vert
            else:
                kalman_gain, cov_dev = filter_normal
#            if msmt[0]==minx:
#                msmt[0] += msmt[2] - avgwidth/2.
#                msmt[1] += msmt[3]/2.
#                kalman_gain, cov_dev = filter_horz
#            elif msmt[0]+msmt[2]==maxx:
#                msmt[0] += avgwidth/2.
#                msmt[1] += msmt[3]/2.
#                kalman_gain, cov_dev = filter_horz
#            elif msmt[1]==miny:
#                msmt[0] += msmt[2]/2.
#                msmt[1] += msmt[3] - avgheight/2.
#                kalman_gain, cov_dev = filter_vert
#            elif msmt[1]+msmt[3]==maxy:
#                msmt[0] += msmt[2]/2.
#                msmt[1] += avgheight/2.
#                kalman_gain, cov_dev = filter_vert
#            else:
#                msmt[:2] += msmt[2:]/2.
#                kalman_gain, cov_dev = filter_normal
            # created this measurement
            include_match = included_match_mtx[:,msmt_idx+1]
            match_1 = match_idxs[msmt_idx]
            match_2 = match_idxs[msmt_idx+1]
            mean_dev = np.einsum(kalman_gain[include_match], [0,1,2],
                                 msmt - means[include_match,:4], [0,2], [0,1])
            new_means[match_1:match_2] = means[include_match] + mean_dev
            new_covs[match_1:match_2] = covs[include_match] + cov_dev[include_match]
            new_samples[match_1:match_2, 0] = 1.
            new_samples[match_1:match_2, 1] = samples[include_match, 1]
            new_exist[match_1:match_2] = match_sample_msmt[include_match,msmt_idx+1]
            new_ids_msmt = ids[include_match]
            if replacing_ids[msmt_idx] >= 0:
                new_ids_msmt[sprouts[include_match], replacing_ids[msmt_idx]] = True
            new_ids[match_1:match_2] = new_ids_msmt
#            # occluded by this measurement # depends on kalmaning
#            include_match = included_match_mtx[:,msmt_idx+n_measurements+1]
#            match_1 = match_idxs[msmt_idx + n_measurements]
#            match_2 = match_idxs[msmt_idx+1 + n_measurements]
#            mean_dev = np.einsum(occlude_kg[include_match], [0,1,2],
#                                 msmt[:2] - means[include_match,:2], [0,2], [0,1])
#            new_means[match_1:match_2] = means[include_match] + mean_dev
#            new_covs[match_1:match_2] = covs[include_match] + occlude_dev[include_match]
#            new_samples[match_1:match_2] = 1.
#            new_exist[match_1:match_2] = match_sample_msmt[include_match,
#                                         msmt_idx+n_measurements+1]
#            new_ids[match_1:match_2] = ids[include_match]
        assert np.all(new_exist < 1+1e-10)
        assert np.all(new_exist > -1e-10)
        object_exist = np.einsum(new_exist, [0], new_ids, [0,1], [1])
        assert np.all(object_exist <= 1)

        
        ## rearrange new and old structures
        old_tracker_structure = tracker_structure
        tracker_structure = new_tracker_structure
        samples, means, covs, existences, ids = tracker_structure
        new_tracker_structure = old_tracker_structure
        new_samples, new_means, new_covs, new_exist, new_ids = new_tracker_structure
        
        
        ## prune via KDTree
        # very similar particles are 'merged' by combining their existences
        prunable = existences > 1e-3/nsamples
        for obj in range(nobjects):
            prune_obj = np.where(prunable & ids[:,obj])[0]
            if prune_obj.shape[0] == 0: continue
            kdmeans = means[prune_obj].copy()
            kdmeans[:,4:] *= fps / skip
            for pair_id1, pair_id2 in cKDTree(kdmeans).query_pairs(1.5, eps=1.):
                existences[prune_obj[pair_id2]] += existences[prune_obj[pair_id1]]
                existences[prune_obj[pair_id1]] = 0
#        prunable = np.where(existences > 1e-3/nsamples)[0]
#        kdmeans = np.empty((prunable.shape[0],7))
#        kdmeans[:,:4] = means[prunable,:4]
#        kdmeans[:,4:6] = means[prunable,4:6] * fps/skip # convert to roughly equal scale
#        kdmeans[:,6] = np.einsum(np.arange(nobjects)*1000,[1],ids[prunable],[0,1],[0])
#        kpairs = cKDTree(kdmeans).query_pairs(1.5, eps=1.)
#        #print "{:d} pairs".format(len(kpairs))
#        for pair_idx1, pair_idx2 in kpairs:
#            existences[prunable[pair_idx2]] += existences[prunable[pair_idx1]]
#            existences[prunable[pair_idx1]] = 0
        
        
        
        ## get estimates
        object_means = np.einsum(means, [0,1], ids, [0,2], existences, [0], [2,1])
        object_exist = np.einsum(existences, [0], ids, [0,1], [1])
        assert np.all(object_exist <= 1)
        object_means[object_exist > 1e-10] /=\
                                object_exist[object_exist > 1e-10,None]
        reports = np.array((unique_ids, object_means[:,0]-object_means[:,2]/2,
                            object_means[:,1]-object_means[:,3]/2,
                            object_means[:,2], object_means[:,3])).T
#        odd = np.einsum(samples,[0,1], ids, [0,2], existences,[0],[2,1])
#        odd[object_exist > 1e-4] /= object_exist[object_exist > 1e-4, None]
        
#        rough_object_cov = np.zeros((nobjects,))
#        for obj_id in range(nobjects):
#            this_obj = ids[:,obj_id]
#            total_ex = np.sum(existences[this_obj])
#            if total_ex < 1e-4: continue
#            rough_cov = np.sum(covs[this_obj,1,1] * existences[this_obj])
#            rough_cov += np.sum((means[this_obj,1]-object_means[obj_id,1])**2 *
#                                existences[this_obj])
#            rough_object_cov[obj_id]  = rough_cov / total_ex
#        
#        for ii in range(nobjects):
#            if object_exist[ii] < .5: continue
#            for jj in range(ii):
#                if object_exist[jj] < .5: continue
#                if np.all(np.abs(object_means[ii] - object_means[jj]) <
#                          np.array([10.,10.,5.,10.,3.,3.])):
##                    if min(object_exist[ii], object_exist[jj]) > .9:
##                        #print "toobig"
##                        continue
##                    #print "merging objects"
##                    if object_exist[ii] < object_exist[jj]:
##                        object_exist[jj] = 1.
##                        object_exist[ii] = 0.
##                    else:
##                        object_exist[jj] = 0.
##                        object_exist[ii] = 1.
#                    important_count += 1
#                    pass
#np.where((object_means[:,0]>1300) & (object_means[:,0]<1700) & (object_means[:,1]>800))[0]
        if show_images: 
            print time
            img = getImg(time)
            img2 = img.copy()
            for measurement in measurements:
                left = measurement[1]
                top = measurement[2]
                bottom = measurement[2] + measurement[4]
                right = measurement[1] + measurement[3]
                drawBox2D(top/2, left/2, bottom/2, right/2, img2,
                          color=[250,100,200])
            plotImg(img2, display_region)
            img2 = img.copy()
            for obj_id, estimate in enumerate(reports):
                if object_exist[obj_id] > .5: boxcolor = [10,10,255]
                elif object_exist[obj_id] > .1: boxcolor = [100,100,255]
                else: continue#boxcolor = [200,200,255]#
                bottom = estimate[2] + estimate[4]
                right = estimate[1] + estimate[3]
                drawBox2D(estimate[2]/2, estimate[1]/2, bottom/2, right/2, img2,
                          color=boxcolor)
            plotImg(img2,display_region)# None)#
#            img2 = img.copy()
#            objects = truth[truth_times[time]:truth_times[time+1], 2:6]
#            for trueobj in objects:
#                drawBox2D(trueobj[1]/2,trueobj[0]/2,trueobj[1]/2+trueobj[3]/2,
#                          trueobj[0]/2+trueobj[2]/2, img2, color=[200,250,100])
#            plotImg(img2, display_region)
    
        if make_video:
            img = getImg(time)
            red_pairwise = np.zeros((nobjects,),dtype=bool)
            for ii in range(nobjects):
                if object_exist[ii] < .5: continue
                for jj in range(ii):
                    if object_exist[jj] < .5: continue
                    if np.all(np.abs(reports[ii,1:]-reports[jj,1:]) < 10):
                        red_pairwise[jj] = True
                        red_pairwise[ii] = True
            for obj_id, estimate in enumerate(reports):
                if object_exist[obj_id] > .5: boxcolor = [10,10,255]
                elif object_exist[obj_id] > .1: boxcolor = [100,100,255]
                else: continue
                if red_pairwise[obj_id]: boxcolor = boxcolor[::-1]
                bottom = estimate[2] + estimate[4]
                right = estimate[1] + estimate[3]
                drawBox2D(estimate[2]/2, estimate[1]/2, bottom/2, right/2, img,
                          color=boxcolor, linewidth=0)
            video_out.writeFrame(img)
        
        if make_score:
            objects = truth[truth_times[time]:truth_times[time+1], 2:6]
            reports_for_scoring = object_exist > .5
            reports_for_scoring &= reports[:,1]+reports[:,3] < maxx+5
            reports_for_scoring &= reports[:,2]+reports[:,4] < maxy+5
            reports_for_scoring &= reports[:,1] > minx-5
            reports_for_scoring &= reports[:,2] > miny-5
            scores += [GOSPA(reports[reports_for_scoring, 1:], objects, c=1)]
            
        if save_detections:
            report_list += [reports[object_exist > .5, :]]
            
            
    if make_video: video_out.close()
    
    if make_score: print np.sum(scores) / truth_times[maxtime] * skip
    
    if save_detections:
        final_report_list = []
        for time, reportez in enumerate(report_list):
            this_report = np.concatenate((
                    np.zeros((reportez.shape[0],1))+time*skip,
                    reportez,
                    np.tile([[1,1,1]], (reportez.shape[0],1)) ), axis=1)
            for thisskip in range(skip):
                final_report_list.append(this_report.copy())
                this_report[:,0] += 1
        report_list = np.concatenate(final_report_list, axis=0)
        np.savetxt(detection_file_name, report_list,
                   #fmt = ['%d']*2 + ['%.1f']*4 + ['%d']*4,
                   fmt = ['%d']*2 + ['%.1f']*4 + ['%d']*3,
                   delimiter=',')