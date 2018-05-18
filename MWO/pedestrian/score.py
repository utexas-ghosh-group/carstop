#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 5/17/18
"""

import numpy as np
import motmetrics
from gospa import GOSPA

truth_file = 'train/MOT17-04-FRCNN/gt.txt'
estimate_file = 'detect_non.txt'


truth = np.loadtxt(truth_file, delimiter=',')

data = np.loadtxt(estimate_file, delimiter=',')
data[:,0] += 1

maxtime = max(np.max(data[:,0]), np.max(truth[:,0]))

true_card = float(truth.shape[0])
data_card = float(data.shape[0])

mmacc = motmetrics.MOTAccumulator(auto_id=True)
gospa = 0.
curr_idx_data = 0
curr_idx_truth = 0
for time in range(int(maxtime)+1):
    next_idx_data = curr_idx_data + np.searchsorted(data[curr_idx_data:,0], time+1)
    X = data[curr_idx_data : next_idx_data]
    curr_idx_data = next_idx_data
    
    next_idx_truth = curr_idx_truth + np.searchsorted(truth[curr_idx_truth:,0], time+1)
    Y = truth[curr_idx_truth : next_idx_truth]
    curr_idx_truth = next_idx_truth
    
    mmacc.update(X[:,1], Y[:,1], 
                 motmetrics.distances.iou_matrix(X[:,2:6], Y[:,2:6], 1.))
    gospa += GOSPA(X[:,2:6], Y[:,2:6])

mh = motmetrics.metrics.create()
summary = mh.compute(mmacc, metrics=['mota','motp','idf1','mostly_tracked','mostly_lost',
                                     'idfp','idfn','num_switches','num_fragmentations'])
print(summary)
print("cardinality error {:.2f}".format(data_card/true_card - 1))
print("GOSPA {:.0f}".format(gospa))
