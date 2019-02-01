#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 8/17/18
"""
import numpy as np
import cv2
import time
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize

#from velodyneLidarRobust import LIDAR as Lidar
from quanergyLidarRobust import LIDAR as Lidar

index = 1
class Camera():
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FPS, 10)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
    def __enter__(self): return self
    def __exit__(self, errtype, errval, traceback): self.cap.release()
    def read(self):
        ret, frame = self.cap.read()
        assert ret
        return frame
    
class Cv2Display():
    def __init__(self): pass
    def __enter__(self): return self
    def __exit__(self, errtype, errval, traceback): cv2.destroyAllWindows()
    def display(self, img):
        cv2.imshow('Lidar Points On Video', img)
        cv2.waitKey(10)
        global index
        #cv2.imwrite('lidar_test/{:05d}.png'.format(index),img)
        index +=1
    

pose = np.loadtxt('calib_extrinsic.txt')
pose[:,:2] *= -1 # lidar was backwards last time?

imfocal = [930., 930.]
imcenter = [360., 650.]

cm_obj = ScalarMappable(norm = Normalize(0, 30.), cmap='jet')

time_start = time.time()

with Cv2Display() as display, Lidar() as lidar, Camera() as camera:
    index = 1
    while True:
        
        # get data
        img = camera.read()
        points = lidar.get(timeout=.1)
        points = np.array(points)
        #img=cv2.imread("data_image/{:05d}.jpg".format(index))
        #points = np.load("data_lidar/{:05d}.npy".format(index))
        index+=1
        # transform lidar points to image reference
        points = points.dot(pose[:3,:3].T) + pose[:3,3]
        dist = points[:,2]
        points = points[:,:2] / points[:,2:3]
        
        # find image points
        points *= imfocal
        points += imcenter
        points = points.astype(int)
        include = np.all(points >= 1, axis=1)
        include &= np.all(points+1 < [720,1280], axis=1)
        include &= dist > 0
        include &= dist < 50
        color = (cm_obj.to_rgba(30.-dist)[:,:3] * 255.9).astype(np.uint8)
        points = points[include]
        color = color[include]
    
        # plot
        for xoff in [-1,0,1]:
            for yoff in [-1,0,1]:
                img[points[:,0]+xoff, points[:,1]+yoff] = color
        display.display(img)
        
        # pause to test functionality
        #time.sleep(10) #
        
        # print timing
        time_end = time.time()
        print("time diff {:.2f}".format(time_end-time_start))
        time_start = time_end
