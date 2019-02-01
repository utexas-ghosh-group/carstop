#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
GUI for highway merge warner
last mod 6/15/18 python 3 compatible, moved all visual display outside of process
"""

display_new_car = 0
display_wait_warning = 1
display_act_warning = 2
display_safe = 3
display_close = 4

from multiprocessing import Process, Pipe
import cv2
from prototype.app_highwaymerge.audioWarning import audioWarning
import time # for pausing

my_folder = __file__[:-11] if __file__[-4:]=='.pyc' else __file__[:-10]
safe_image = cv2.imread(my_folder + 'demo_safe.png')
wait_warning_images = (cv2.imread(my_folder + 'demo_waitwarn0.png'),
                       cv2.imread(my_folder + 'demo_waitwarn1.png'))
wait_image_period = 2
act_warning_image = cv2.imread(my_folder + 'demo_actwarn.png')

class GUI():
    def __init__(self, video=True):
        if video:
            # have two run two images, or else it doesn't show up reliably
            cv2.imshow("Merge Alert", safe_image)
            cv2.waitKey(50)
            cv2.imshow("Merge Alert", safe_image)
            cv2.waitKey(1)
    
        receiver, sender = Pipe(False)
        self.process = Process(target=audioWarning, args=(receiver,))
        self.sender=sender
        self.receiver=receiver
        
        self.video=video
        self.warning_img_counter = 0

    def startDisplay(self):
        print("beginning of startDisplay")
        self.process.start()
        print("end of startDisplay")
        
    def stopDisplay(self):
        print("stop display called")
        if self.video:
            cv2.destroyWindow("Merge Alert")
        self.sender.send(display_close)
        time.sleep(.2)
        self.sender.close()
        self.process.join()
        self.receiver.close()
        
    def changeDisplay(self, msg):
        self.sender.send(msg)
        
        if self.video:
            if msg == display_safe:
                cv2.imshow("Merge Alert", safe_image)
            elif msg == display_new_car:
                cv2.imshow('Merge Alert', wait_warning_images[0])
                self.warning_img_counter = 0
            elif msg == display_wait_warning:
                cv2.imshow('Merge Alert', wait_warning_images[
                                            self.warning_img_counter//2%2])
                self.warning_img_counter += 1
            elif msg == display_act_warning:
                cv2.imshow('Merge Alert', act_warning_image)
            if msg == display_close: cv2.destroyWindow("Merge Alert")
