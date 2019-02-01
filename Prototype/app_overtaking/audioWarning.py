#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 6/15/18
"""

display_new_car = 0
display_wait_warning = 1
display_act_warning = 2
display_safe = 3
display_close = 4

import wave
import pyaudio
import time # for pausing

def reverseSound(sound): return b''.join((sound[k*2:k*2+2] for k in
                                         range(len(sound)//2-1,-1,-1)))

sample_width = 2
n_channels = 1 # mono
framerate = 44100 # kHz, standard
period = 22050
# load sounds from .wav files as binary strings
my_folder = __file__[:-16] if __file__[-4:]=='.pyc' else __file__[:-15]
wf = wave.open(my_folder + 'newcar_warning.wav', 'rb')
bwoop_period = wf.getnframes() // period
bwoop_sound = wf.readframes(bwoop_period * period)
wf.close()
safe_sound = reverseSound(bwoop_sound)
safe_period = bwoop_period
wait_warning_sound = bwoop_sound[22050*2:44100*2]
wait_sound_period = len(wait_warning_sound) // (sample_width * period)
act_warning_sound = bwoop_sound[33075*2:44100*2] +\
                    reverseSound(bwoop_sound[33075*2:44100*2])
act_warning_period = len(act_warning_sound) // (sample_width * period)
newcar_sound = bwoop_sound[33075*2:44100*2]*2
newcar_sound_period = bwoop_period // 2


def audioWarning(receiver):
    #print("GUI process name "+__name__)
    p = pyaudio.PyAudio()
    stream = p.open(format = p.get_format_from_width(sample_width),
                    channels = n_channels,
                    rate = framerate,
                    output = True)
    
    currently_act_warning = False
    currently_wait_warning = False
    act_sound_pos = 0
    wait_sound_pos = 0
    
    while True:
        single_newcar = False
        single_safe = False
        
        msg = -1
        while receiver.poll():
            msg = receiver.recv()
            
            if msg == display_new_car:
                single_newcar = True
            elif msg == display_close:
                    stream.stop_stream()
                    stream.close()
                    p.terminate()
                    return
            assert msg >= 0 and msg < 4
            
        if msg == display_wait_warning:
            currently_act_warning = False
            currently_wait_warning = True
        elif msg == display_act_warning:
            currently_wait_warning = False
            currently_act_warning = True
        elif msg == display_safe:
            if (currently_act_warning or currently_wait_warning):
                single_safe = True # make a one-time noise for switching to safe
            single_newcar = False # don't make new car noise if it's safe right now
            currently_act_warning = False
            currently_wait_warning = False
        
        if single_newcar:
            stream.write(newcar_sound) # play whole thing (1 s)
        elif single_safe:
            stream.write(safe_sound)
        elif currently_act_warning:
            stream.write(act_warning_sound[act_sound_pos*period*sample_width:
                                               (act_sound_pos+1)*period*sample_width])
            act_sound_pos += 1
            if act_sound_pos == act_warning_period:
                act_sound_pos = 0
        elif currently_wait_warning:
            stream.write(wait_warning_sound[wait_sound_pos*period*sample_width:
                                               (wait_sound_pos+1)*period*sample_width])
            wait_sound_pos += 1
            if wait_sound_pos == wait_sound_period:
                wait_sound_pos = 0
        else:
            time.sleep(.2)




### testing
if __name__ == '__main__':
    
    class FakePoll():
        def __init__(self):
            self.switch = False
        def poll(self):
            self.switch = not self.switch
            return self.switch
        def recv(self):
            return input('send msg')
    
    audioWarning(FakePoll())
