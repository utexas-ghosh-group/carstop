#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 5/17/18
multi-bernoulli filter, track-oriented (aka labelled)
"""

data_folder = 'train/MOT17-04-FRCNN/'
fps=30
skip=2 # only use every other frame
nsamples = 2048
nobjects = 72
occlusion = "None"

show_images = False
display_region = [300,0,540,400]#[0,200,300,900] # if None, display whole image, else (t,l,b,r)
make_video = False
video_name = 'track_test.mkv'
make_score = False
save_detections = True
detection_file_name = 'detect_non.txt'

import numpy as np
from scipy.misc import imread
from scipy.spatial import cKDTree
from skvideo.io import FFmpegWriter as vwriter
import matplotlib.pyplot as plt
import trackmodel as tm
from gospa import GOSPA
from time import time as timer


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

# data association approximation, from reference [15]
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
    min_confidence = np.min(data[:,5])
    times = list(np.where(np.diff(data[:,0])!=0)[0]+1)
    times = np.array([0] + times + [data.shape[0]], dtype=int)
        
    maxtime = times.shape[0]-1 # 66#

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
    samples = np.zeros((nsamples, tm.sample_nft)) + tm.initial_sample
    existences = np.zeros((nsamples,))
    existences[0] = tm.initial_cardinality
    ids = np.zeros((nsamples, nobjects), dtype=bool)
    tracker_structure = (samples, existences, ids)
    
    new_samples = np.empty(samples.shape)
    new_exist = np.empty(existences.shape)
    new_ids = np.empty(ids.shape, dtype=bool)
    new_tracker_structure = (new_samples, new_exist, new_ids)
    
    # unique labels, because id columns will be reused
    # assigned at first time object is recognized
    unique_ids = np.zeros((nobjects,), dtype=int)
    unique_id_count = 0
        
    worst_time = 0.
    first_time = timer()

    for time in range(0,maxtime,skip):
        starttime = timer()
        measurements = data[times[time]:times[time+1]]
        n_measurements = len(measurements)
        
        # predict
        tm.predict(samples)
        existences *= tm.survival(samples)
        #tm.debug(samples[existences > 1e-10])
        
        # entry
        # so this is hacky... but given the limited number of components
        # and the fact that they aren't easily merged, it is better to not
        # include entry components every single time
        # instead, add components every other time with twice the magnitude
        # I should have made this clearer/added some hyperparameters to control
        if time % 2 == 0:
            for entry_idx, entry_exist in enumerate(tm.entry_cardinality):
                min_exist = np.argmin(existences)
                if existences[min_exist] < entry_exist:
                    samples[min_exist] = tm.entry_samples[entry_idx]
                    ids[min_exist] = False
                    existences[min_exist] = entry_exist
        #debug_object_exist = np.einsum(existences, [0], ids, [0,1], [1])
        #assert np.all(debug_object_exist <= 1)
        #assert np.all(existences >= 0)
        #tm.debug(samples[existences > 1e-10])
            
        ## update
        # false positive likelihood (with generation intensity included)
        fp_msmt = 2e-10 / measurements[:,5]**2
        
        # detection with occlusion
        detect_prob = tm.detect(samples)
        sample_miss_pure = 1 - detect_prob
        if occlusion == 'None':
            pass
        elif occlusion == 'MWO':
            measurements_occluding = tm.prepMeasurements4Occlusion(measurements)
            measurement_existences = np.ones((measurements.shape[0],))
            sample_miss_pure += detect_prob *\
                        tm.occlude(samples, measurements_occluding, measurement_existences)
        elif occlusion == 'OWO':
            object_means = np.einsum(samples, [0,1], ids, [0,2], existences, [0], [2,1])
            object_exist = np.einsum(existences, [0], ids, [0,1], [1])
            #assert np.all(object_exist <= 1)
            include_occluding = object_exist > .1
            object_means[include_occluding] /= object_exist[include_occluding,None]
            
            occluding_objects = tm.prepObjects4Occlusion(object_means[include_occluding])
            occluding_exist = object_exist[include_occluding]
#            detect_prob *= 1 - tm.occlude(samples, occluding_objects, occluding_exist)
            detect_prob *= 1 - tm.occludeOWO2(samples, ids, np.where(include_occluding)[0],
                                           occluding_objects, occluding_exist)
            
            sample_miss_pure[:] = 1 - detect_prob
            
        #assert np.all(detect_prob > -1e-10)
        #assert np.all(detect_prob < 1+1e-10)
        #assert np.all(sample_miss_pure > -1e-10)
        #assert np.all(sample_miss_pure < 1+1e-10)
        
        # parallel likelihood determination
        sample_miss = existences * sample_miss_pure
        sample_msmt = np.tile((existences * detect_prob)[:,None], (1,n_measurements))
        prep = tm.prepLikelihood(samples)
        for msmt_idx, measurement in enumerate(measurements):
            sample_msmt[:,msmt_idx] *= tm.likelihood(samples, prep, measurement)
        
        # reduce to object likelihoods
        object_msmt = np.einsum(sample_msmt, [0,1], ids, [0,2], [2,1])
        object_exist = np.einsum(existences, [0], ids, [0,2], [2])
        object_miss = 1 - object_exist + np.einsum(sample_miss, [0], ids, [0,2], [2])
        #assert np.all(object_miss >= 0)
        #assert np.all(object_miss <= 1)
        #assert np.all(object_exist < 1.02)
        sprouts = np.any(ids,axis=1)==False # poisson generated terms
        miss_msmt = fp_msmt + np.sum(sample_msmt[sprouts],axis=0)
        #tm.debug(samples[existences > 1e-10])
        
        # data association
        jam_object_msmt, jam_object_miss, jam_miss_msmt = DA_JAM(object_msmt,
                                                             object_miss, miss_msmt)
        #assert np.all(jam_object_msmt < 1+1e-10)
        #assert np.all(jam_object_miss < 1+1e-10)
        #assert np.all(jam_miss_msmt < 1+1e-10)
        
        # expand to sample assignment probabilities
        match_object_msmt = jam_object_msmt.copy()
        match_object_msmt[object_msmt > 1e-30] /= object_msmt[object_msmt > 1e-30]
        match_object_miss = jam_object_miss.copy()
        match_object_miss[object_miss > 1e-30] /= object_miss[object_miss > 1e-30]
        match_miss_msmt = jam_miss_msmt.copy()
        match_miss_msmt[miss_msmt > 1e-30] /= miss_msmt[miss_msmt > 1e-30]
        match_sample_msmt = np.concatenate((sample_miss[:,None],
                                            sample_msmt), axis=1)
        for obj in range(nobjects):
            match_sample_msmt[ids[:,obj],1:n_measurements+1] *= match_object_msmt[obj,:]
            match_sample_msmt[ids[:,obj],0] *= match_object_miss[obj]
        match_sample_msmt[sprouts,1:n_measurements+1] *= match_miss_msmt
        #debug_object_match = np.einsum(match_sample_msmt, [0,1], ids, [0,2], [2,1])
        #assert np.all(debug_object_match < 1+1e-10)
        #assert np.all(match_sample_msmt > -1e-10)
        
        # find the top entries in match_sample_msmt
        percentile = 100.*n_measurements/(n_measurements + 1)
        min_value = np.percentile(match_sample_msmt, percentile, interpolation='lower')
        included_match_mtx = match_sample_msmt > min_value
        remaining_samples = nsamples - np.sum(included_match_mtx)
        identicals_samp, identicals_msmt = np.where(match_sample_msmt == min_value)
        included_match_mtx[identicals_samp[:remaining_samples],
                           identicals_msmt[:remaining_samples]] = True
        match_idxs = np.cumsum(np.sum(included_match_mtx, axis=0))
        #assert match_idxs[-1] == nsamples
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
        replaced_ids = objects_ordered[:n_new_ids]
        replacing_ids = {msmt_idx:-1 for msmt_idx in range(n_measurements)}
        for new_id_num in range(n_new_ids):
            replacing_ids[new_msmts_ordered[-new_id_num-1]] = replaced_ids[new_id_num]
        ids[:,replaced_ids] = False
        unique_ids[replaced_ids] = range(n_new_ids)
        unique_ids[replaced_ids] += unique_id_count
        unique_id_count += n_new_ids
        
        # update
        include_nomatch = included_match_mtx[:,0]
        tm.updateOnMiss(samples[include_nomatch], sample_miss_pure[include_nomatch],
                        measurements, out = new_samples[:match_idxs[0]])
        new_exist[:match_idxs[0]] = match_sample_msmt[include_nomatch,0]
        new_ids[:match_idxs[0]] = ids[include_nomatch]
        prep = tm.prepUpdate(samples)
        #assert not np.any(np.isnan(new_samples[:match_idxs[0]]))
        for msmt_idx, measurement in enumerate(measurements):
            # created this measurement
            include_match = included_match_mtx[:,msmt_idx+1]
            match_1 = match_idxs[msmt_idx]
            match_2 = match_idxs[msmt_idx+1]
            tm.update(samples[include_match], prep[include_match], measurement,
                      out = new_samples[match_1:match_2])
            new_exist[match_1:match_2] = match_sample_msmt[include_match,msmt_idx+1]
            new_ids_msmt = ids[include_match]
            if replacing_ids[msmt_idx] >= 0:
                new_ids_msmt[sprouts[include_match], replacing_ids[msmt_idx]] = True
            new_ids[match_1:match_2] = new_ids_msmt
        #assert not np.any(np.isnan(new_samples))
        #tm.debug(new_samples[new_exist > 1e-10])
        #assert np.all(new_exist > -1e-10)
        #debug_object_exist = np.einsum(new_exist, [0], new_ids, [0,1], [1])
        #assert np.all(debug_object_exist <= 1)

        
        ## rearrange new and old structures
        old_tracker_structure = tracker_structure
        tracker_structure = new_tracker_structure
        samples, existences, ids = tracker_structure
        new_tracker_structure = old_tracker_structure
        new_samples, new_exist, new_ids = new_tracker_structure
        
        
        ## prune via KDTree
        # very similar particles are 'merged' by combining their existences
        if time % 1 == 0:
            prunable = existences > 1e-3/nsamples
            for obj in range(nobjects):
                prune_obj = np.where(prunable & ids[:,obj])[0]
                if prune_obj.shape[0] == 0: continue
                kdmeans = samples[prune_obj][:,[0,2,9,11,1,10]].copy()
                kdmeans[:,4:] *= fps / skip
                for pair_id1, pair_id2 in cKDTree(kdmeans).query_pairs(1., eps=1.):
                    existences[prune_obj[pair_id2]] += existences[prune_obj[pair_id1]]
                    existences[prune_obj[pair_id1]] = 0
            #n_meaningful_samples = np.sum(prunable)
            #n_meaningful_samples_new = np.sum(existences > 1e-3/nsamples)
            #print "prune {:d}/{:d}".format(n_meaningful_samples_new, n_meaningful_samples)
        
        
        
        ## get estimates
        object_means = np.einsum(samples, [0,1], ids, [0,2], existences, [0], [2,1])
        object_exist = np.einsum(existences, [0], ids, [0,1], [1])
        assert np.all(object_exist <= 1)
        object_means[object_exist > 1e-10] /=\
                                object_exist[object_exist > 1e-10,None]
        reports = np.append(unique_ids[:,None], tm.output(object_means), axis=1)
        worst_time = max(worst_time, timer() - starttime)
        
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
            plotImg(img2,display_region)
    
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
            scores += [GOSPA(reports[reports_for_scoring, 1:], objects, c=1)]
            
        if save_detections:
            report_list += [reports[object_exist > .5, :]]
            
    final_time = timer() - first_time            

    if make_video: video_out.close()
    
    if make_score: print np.sum(scores) / truth_times[maxtime] * skip
    
    if save_detections:
        final_report_list = []
        for timez, reportez in enumerate(report_list):
            this_report = np.concatenate((
                    np.zeros((reportez.shape[0],1))+timez*skip,
                    reportez,
                    np.tile([[1,1,1]], (reportez.shape[0],1)) ), axis=1)
            for thisskip in range(skip):
                final_report_list.append(this_report.copy())
                this_report[:,0] += 1
        report_list = np.concatenate(final_report_list, axis=0)
        np.savetxt(detection_file_name, report_list,
                   fmt = ['%d']*2 + ['%.1f']*4 + ['%d']*3,
                   delimiter=',')
        
    print "avg time {:f}".format(final_time / maxtime)
    print "worst time {:f}".format(worst_time)