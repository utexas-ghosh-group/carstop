#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 11/2/18
"""

import numpy as np
from prototype.app_intersection.quanergyPoint import LIDAR
from time import sleep

name = 'whiteboard/'

from multiprocessing import freeze_support
if __name__ == '__main__':
    freeze_support()

with LIDAR() as lidar:
    sleep(20.)
    print("gotime")
    for k in range(300):
        points = lidar.get()
        full_name = "{:s}{:03d}.npy".format(name, k)
        np.save(full_name, points)
