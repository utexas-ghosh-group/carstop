#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 12/3/17 wide-range mode on

function to call: get(timeout in seconds)
output: list of two-element x-y tuples

If run on its own, plots detected points
"""
from multiprocessing import Process, Queue
from math import cos, sin
import cantools
import prototype.sensors.canlib as canlib

# the .dbc file is a translator for CAN messages
my_folder = __file__[:-14] if __file__[-3:]=='.py' else __file__[:-15]
db = cantools.db.load_file(my_folder + 'proprietary/radar_can_defs.dbc')
initialize_msg = db.messages[126]
initialize_params = initialize_msg.decode([0]*8)
initialize_params['CAN_RX_RADAR_CMD_RADIATE'] = 'On'
initialize_params['CAN_RX_MAXIMUM_TRACKS'] = 64
# if Filtered, will delete and merge detections by Delphi's rules
# in my experience, this results in limited accuracy
initialize_params['CAN_RX_RAW_DATA_ENABLE'] = 'Raw'#'Filtered'#
# if On, only use long-and-narrow beam
initialize_params['CAN_RX_LR_ONLY_TRANSMIT'] = 'Off'#'On'#


""" This is a shortened version of the code used by 2016's senior design
team. I imagine they got the original from Delphi or AutonomouStuff at some point.
The code has been shortened to only gather object location information, 
namely range, angle, range rate (speed toward/from self) and lateral rate"""
class Radar(Process):
    """ Listens for new Radar messages over CAN and parses for the dispatcher.

    This parser reads messages from the CAN Bus using the Kvaser USB Python SKD
    and formats Radar information into a python object. Then we send the data
    along to the event dispatcher.
    """
    def __init__(self, filename):
        Process.__init__(self)
        self.queue = Queue()
        
        self.init_params = initialize_params
        self.init_msg = initialize_msg
        cl = canlib.canlib()
        self.ch1 = cl.openChannel(0, canlib.canOPEN_ACCEPT_VIRTUAL)
        
        
    def __enter__(self):
        return self.queue
        
    def terminate(self):
        self.ch1.busOff()
        self.ch1.close()
        self.queue.close()
        super(Radar, self).terminate()
        
    def __exit__(self, errtype=None, errval=None, traceback=None):
        self.terminate()

    def run(self):
        print("Using channel: %s, EAN: %s" % (
            self.ch1.getChannelData_Name(), self.ch1.getChannelData_EAN()))

        self.ch1.setBusOutputControl(canlib.canDRIVER_NORMAL)
        self.ch1.setBusParams(canlib.canBITRATE_500K)
        self.ch1.busOn()

        # Initialize the Radar
        self.ch1.writeWait(self.init_msg.frame_id,
                        self.init_msg.encode(self.init_params), 8, 1000)

        points = []
        
        while True:
            msgId, msg, dlc, flg, msgtime = self.ch1.read(1000)

            if msgId >= 1280 and msgId <= 1343:
                output = db.decode_message(msgId, msg)
                if output['CAN_TX_TRACK_RANGE'] == 0: continue
                angle_rad = -.01745 * output['CAN_TX_TRACK_ANGLE']
                points.append((output['CAN_TX_TRACK_RANGE']*cos(angle_rad),
                                output['CAN_TX_TRACK_RANGE']*sin(angle_rad)))
                    
            elif msgId == 1248:
                self.queue.put(tuple(points))
                points = []
                



if __name__ == '__main__':
    # plot radar points
    import numpy as np
    import cv2
    
    size = 320
    distance = 20.
    
    base_image = np.zeros((size*2, size*2, 3), dtype=np.uint8) + 255
    base_image[size-5:size+5, size-5:size+5] = [230,230,230]
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
            
    color = (0,0,0)
    
    offsets = ((-1,0),(1,0),(0,-1),(0,1),(0,0))
    
    with Radar() as lidar, Display() as display:
        while True:
            cloud = lidar.get()
            
            cloud = np.array(cloud)
            cloud *= -size/distance
            cloud[:,0] += size*2
            cloud[:,1] += size
            include = np.all((cloud>=0) & (cloud < size*2), axis=1)
            cloud = cloud[include].astype(int)
            img = base_image.copy()
            for offx, offy in offsets:
                img[cloud[:,0]+offx, cloud[:,1]+offy] = color
            display.display(img)
