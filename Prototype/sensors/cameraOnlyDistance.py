#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 8/29/18
uses the detection box from a camera, and knowledge of the camera's position,
to estimate the location of the detected object on the ground

box = [top, left, bottom, right] pixels, can accept single boxes or a matrix
cam_focal and cam_center = (horizontal, vertical)
cam_pose = 4x4 pose matrix, dimensions: forward, left, up
pixel numbers in openCV style, (up-to-down, left-to-right)

output: coordinates (forward, left, up)
"""
import numpy as np

def box2Position(box, cam_pose, cam_focal, cam_center):
    Rx, Ry, Rz = cam_pose[:3,:3]
    tx, ty, tz = cam_pose[:3,3]
    
    h = box[...,2]
    w = (box[...,1]+box[...,3])/2.
    h = (cam_center[1] - h)/cam_focal[1]
    w = (cam_center[0] - w)/cam_focal[0]
    A_X = Rz-h*Rx
    A_Y = Ry-w*Rx
    A_Z = np.zeros(A_X.shape)
    A_Z[...,2] = -1
    A = np.array((A_X, A_Y, A_Z))
    b_X = h*tx-tz
    b_Y = w*tx-ty
    if type(b_X) == np.ndarray:
        b_Z = np.zeros(b_X.shape)
    else:
        b_Z = 0.
    b = np.array((b_X, b_Y, b_Z))
    return np.linalg.solve(A, b)