# -*- coding: utf-8 -*-
"""
white square image
circle at the center of the image
points where specified by point cloud
"""

import os
import cv2
import numpy as np
from time import sleep

name = 'whiteboard/'
files = sorted(os.listdir(name))

imgsize = 320 # 640x640 image
realsize = 8. # length/width of considered area in real coordinates (ex. meters)
circlesize = 1./realsize*imgsize
circlecolor = (250,210,190)
pointcolor = (0,0,0) # black

# size and shape of points
## square
#point_size = 1
#offsx = range(-point_size, point_size+1)
#offsy = range(-point_size, point_size+1)
#offsets = tuple([(offx, offy) for offx in offsx for offy in offsy])
# cross
offsets = ((-1,0),(1,0),(0,-1),(0,1),(0,0))

def plotCircle(img, x, y, radius, color):
    x_min = int(x+radius) - 1
    x_min = max(x_min, 0)
    x_max = int(x-radius) + 1
    x_max = min(x_max+1, imgsize*2)
    y_min = int(y+radius) - 1
    y_min = max(y_min, 0)
    y_max = int(y-radius) + 1
    y_max = min(y_max+1, imgsize*2)
    
    x_off = x - np.arange(x_min, x_max)
    y_off = y - np.arange(y_min, y_max)
    x_off, y_off = np.meshgrid(y_off, x_off)
    include = np.hypot(x_off, y_off) < radius + .5/imgsize
    img[x_min:x_max, y_min:y_max][include] = color

base_image = np.zeros((imgsize*2, imgsize*2, 3), dtype=np.uint8) + 255
plotCircle(base_image, imgsize, imgsize, circlesize, circlecolor)


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


img = base_image.copy()

with Display() as display:
    for filename in files:
        cloud = np.load(name + filename)
        #cloud = cloud[:,:2] # if point cloud is 3D
        
        # convert directions if necessary
        # for this data, +x is up and +y is left
        # opencv, and most plotters, say +x is down and +y is right
        cloud *= -1
        
        cloud *= imgsize/realsize
        cloud += imgsize
        # remove points that are not in image
        # actually, make sure that you leave a buffer for your points' shapes
        # otherwise, you might for instance ask to color row -1
        include = np.all((cloud>=1) & (cloud < imgsize*2-1), axis=1)
        cloud = cloud[include].astype(int)
        
        img[:] = base_image
        for offx, offy in offsets:
            img[cloud[:,0]+offx, cloud[:,1]+offy] = pointcolor
        display.display(img)
        sleep(.02)