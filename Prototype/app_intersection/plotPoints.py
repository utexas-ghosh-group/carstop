# -*- coding: utf-8 -*-
"""
last mod 11/1/18
"""
import numpy as np
from realtimeLidarEdge import LIDAR
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize

plot_period = 1 # only plot every so often if speed is issue...

cm_obj = ScalarMappable(norm = Normalize(0, 30), cmap='jet')
#ScalarMappable(Normalize(0,7))



#import matplotlib.pyplot as plt
#fig1 = plt.figure(figsize=(8., 8.))
#fig1.subplots_adjust(left=0, bottom=0, right=1, top=1, wspace=None, hspace=None)
#ax = fig1.gca()
#ax.set_xlim([-20,20])
#ax.set_ylim([-20,20])
#plt.margins(0,0)
#mycar = ax.add_patch(plt.Rectangle((-3.6,-1),5.,2.,0.,fill=True, color='k'))
##scatterpoints = ax.scatter([], [], 2., color=[],
##                             cmap = )
#scatterpoints = ax.scatter([], [], 2., color = (0,0,240))
#fig1.canvas.draw()
#plt.show(block=False)
#fig1.canvas.update()
#fig1.canvas.flush_events()

## for updating
#while plt.fignum_exists(fig1.number):
#    scatterpoints.set_offsets(np.array(cloud))
#    #scatterpoints.set_array(loglist[:,2])
#    ax.draw_artist(ax.patch)
#    ax.draw_artist(mycar)
#    ax.draw_artist(scatterpoints)
#    fig1.canvas.update()
#    fig1.canvas.flush_events()
    


import cv2
base_image = np.zeros((480,480,3), dtype=np.uint8)
class Display():
        def __init__(self): pass
        def __enter__(self):
            cv2.imshow('lidar side detections', base_image)
            cv2.waitKey(5)
            return self
        def __exit__(self, a, b, c): cv2.destroyWindow('lidar side detections')
        def display(self, image):
            cv2.imshow('lidar side detections', image)
            cv2.waitKey(5)
colors = (cm_obj.to_rgba(range(30))[:,:3] * 255.9).astype(np.uint8)
color = (0,0,240)



with LIDAR() as lidar, Display() as display:
    while True:
        cloud = lidar.get()
        
        cloud = np.array(cloud).astype(int)
        img = base_image.copy()
        for offx in xrange(-1,2):
            for offy in xrange(-1,2):
                img[cloud[:,0]+offx, cloud[:,1]+offy] = color
        display.display(img)