#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 20 17:28:56 2018

@author: motrom
"""

import wave
import pyaudio
import time # for pausing
from multiprocessing import Process, Value
from ctypes import c_bool

sample_width = 1
n_channels = 1 # mono
framerate = 44100 # kHz, standard
# load sounds from .wav files as binary strings
my_folder = __file__[:-18] if __file__[-3:]=='.py' else __file__[:-19]
wf = wave.open(my_folder + 'beepbeepbeep.wav', 'rb')
beep_sound = wf.readframes(17640) # 2/5 second
wf.close()


class AudioWarning(Process):
    def __init__(self):
        Process.__init__(self)
        self.sound = beep_sound
        self.going = Value(c_bool)
        self.going.value = True
        self.warning = Value(c_bool)
        self.warning.value = False
        
    def __enter__(self):
        Process.start(self)
        return self
    
    def change(self, warning):
        self.warning.value = warning

    def run(self):
        p = pyaudio.PyAudio()
        stream = p.open(format = p.get_format_from_width(sample_width),
                            channels = n_channels,
                            rate = framerate,
                            output = True)
    
        while self.going.value:
            if self.warning.value:
                stream.write(self.sound) # play whole thing (.5 s)
            else:
                time.sleep(.05)
                
        stream.stop_stream()
        stream.close()
        p.terminate()
        print("closed audio")

    def __exit__(self, errtype=None, errval=None, traceback=None):
        self.terminate()
        
    def terminate(self):
        self.going.value = False
        #super(AudioWarning, self).terminate()
        super(AudioWarning, self).join(1.)
        assert not self.is_alive()


### testing
if __name__ == '__main__':
    
    f = AudioWarning()
    with f:
        for k in range(6):
            f.change(True)
            time.sleep(.3)
            f.change(False)
            time.sleep(.6)
