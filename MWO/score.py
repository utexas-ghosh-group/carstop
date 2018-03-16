#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
fdfd
"""
import numpy as np
import motmetrics
import scipy.optimize as spopt


truth_file = 'train/MOT17-04-FRCNN/gt.txt'
estimate_file = 'detect_train4O.txt'


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

truth = np.loadtxt(truth_file, delimiter=',')
truth_times = list(np.where(np.diff(truth[:,0])>0)[0]+1)
truth_times = np.array([0] + truth_times + [truth.shape[0]], dtype=int)

data = np.loadtxt(estimate_file, delimiter=',')
data_times = list(np.where(np.diff(data[:,0])>0)[0]+1)
data_times = np.array([0] + data_times + [data.shape[0]], dtype=int)

maxtime = len(truth_times) - 1
assert len(data_times) - 1 == maxtime

true_card = float(truth.shape[0])
data_card = float(data.shape[0])

mmacc = motmetrics.MOTAccumulator(auto_id=True)
gospa = 0.
for time in range(maxtime):
    X = data[data_times[time]:data_times[time+1]]
    Y = truth[truth_times[time]:truth_times[time+1]]
    mmacc.update(X[:,1], Y[:,1], 
                 motmetrics.distances.iou_matrix(X[:,2:6], Y[:,2:6], 1.))
    gospa += GOSPA(X[:,2:6], Y[:,2:6])
mh = motmetrics.metrics.create()
summary = mh.compute(mmacc, metrics=['mota', 'motp', 'idr', 'idp', 'idf1', 
                                     'mostly_tracked', 'num_switches'], name='acc')
print(summary)
print("cardinality error {:.2f}".format(data_card/true_card - 1))
print("GOSPA {:.0f}".format(gospa))


#motmetrics.utils.compare_to_groundtruth(B, A)
id_count = -1
new_id_count = -1
ditch_ids = []
include = np.ones((data.shape[0],), dtype=bool)
for time in range(2,maxtime,2):
    X = data[data_times[time]:data_times[time+1],1:]
    new_all = X[:,0] > id_count
    new_new = X[:,0] > new_id_count

    n_new = sum(new_all)
    new_idx = np.where(new_all)[0]
    id_order = np.argsort(X[new_all,0])
    Y = X[new_all][id_order]
    ditch = np.zeros((n_new,),dtype=bool)
    for i in range(n_new):
        if ditch[i]: continue
        obj_i = Y[i]
        for j in range(i+1, n_new):
            obj_j = Y[j]
            if IoU(obj_i[1:], obj_j[1:]) > .75:
                ditch[j] = True
    ditch_ids += list(Y[ditch,0])
            
    id_count = new_id_count
    new_id_count = max(id_count, max(X[:,0]))
    
    ditched = np.array([x[0] in ditch_ids for x in X])
    ditched_idx = np.where(ditched)[0]
    really_include = include[data_times[time]:data_times[time+1]]
    safe_idx = np.where(ditched == False)[0]
    for i in range(ditched_idx.shape[0]):
        for j in range(safe_idx.shape[0]):
            if IoU(X[ditched_idx[i],1:], X[safe_idx[j],1:]) > .75:
                really_include[ditched_idx[i]] = False