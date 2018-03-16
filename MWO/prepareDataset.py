#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar  7 10:58:23 2018

@author: motrom
"""
import numpy as np

data_folder = 'train/MOT17-09-FRCNN/'

data = np.loadtxt(data_folder + 'det/det.txt', delimiter=',')
data = data[:,[0,2,3,4,5,6]]
data = data[np.argsort(data[:,0])]
assert np.unique(data[:,0]).shape[0] == np.max(data[:,0])
assert np.all((np.diff(data[:,0]) == 0) | (np.diff(data[:,0]) == 1))

np.savetxt(data_folder+'det.txt', data, fmt = ['%d'] + ['%.1f']*4 + ['%.2f'],
                   delimiter=',')

data = np.loadtxt(data_folder + 'gt/gt.txt', delimiter=',')
data = data[data[:,6]==1] # objects that are included in scoring
data = data[(data[:,7]==1)|(data[:,7]==2)|(data[:,7]==7)] # people
#data = data[data[:,8] > .25] # people that are at least 25% visible
data = data[:,:6]
data = data[np.argsort(data[:,0])]
np.savetxt(data_folder+'gt.txt', data, fmt='%d', delimiter=',')