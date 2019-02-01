#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 6/20/18 moved cv2 import to __main__ only
the Camera class is set up with __enter__ and __exit__ commands to utilize Python's 'with' statement.
The 'with' statement makes sure that __exit__ is called reliably, even if the rest of the code fails.
This includes keyboard interrupts (ctrl-c) but not necessarily shutting down the terminal or IDE.
Try to exit with ctrl-c so things aren't still running in the background.

For the example usage, Opencv is used to display the video in real time.
"""

import numpy as np
import subprocess, time

def findC920Webcam():
    strout = subprocess.check_output(('v4l2-ctl','--list-devices'))
    webcam_line = strout.find('HD Pro Webcam C920')
    assert webcam_line >= 0, "v4l didn't find the Logitech webcam"
    dev_number_location = strout[webcam_line:].find('/dev/video') + 10
    return int(strout[dev_number_location])

class Camera():
    def __init__(self, remotecompress=False):
        camnumber = findC920Webcam()
        
        if remotecompress:
            informat = 'MJPG' #H264
        else:
            informat = 'YUYV'
        format_command = ['v4l2-ctl','-d',str(camnumber),'-v']
        format_command += ["width=1280,height=720,pixelformat='"+informat+"'"]
        subprocess.check_call(format_command)
        
        no_focus_command = ['v4l2-ctl','-d',str(camnumber),'-c','focus_auto=0']
        subprocess.check_call(no_focus_command)
        
        cmd = ['ffmpeg','-y','-f','video4linux2','-r','10']
        if remotecompress:
            cmd += ['-input_format','mjpeg']#'h264']
        cmd += ['-i','/dev/video'+str(camnumber)]
        cmd += ['-an']
        cmd += ['-c:v','rawvideo','-f','image2pipe','-r','10']
        cmd += ['-pix_fmt', 'bgr24', '-']
        self.run_command = cmd
        
    def read(self):
        frame = self.vid.stdout.read(1280*720*3)
        if len(frame) == 1280*720*3:
            return np.fromstring(frame, dtype=np.uint8).reshape((720,1280,3))
        
    def __enter__(self):
        self.vid = subprocess.Popen(self.run_command, bufsize=4194304,
                                    stdin = subprocess.PIPE,
                                    stdout = subprocess.PIPE,
                                    stderr = None)
        return self

    def __exit__(self, errtype, errval, traceback):
        self.vid.stdin.write('q')
        time.sleep(.1)
        self.vid.stdin.close()
        self.vid.stdout.read()
        time.sleep(.1)
        self.vid.stdout.close()
        self.vid.wait()

        
if __name__ == '__main__':
    import cv2
    with Camera(True) as cam:
        while True:
            frame = cam.read()
            cv2.imshow('hi',frame)
            cv2.waitKey(50)
    cv2.destroyAllWindows()