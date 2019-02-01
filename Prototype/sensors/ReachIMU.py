# -*- coding: utf-8 -*-
"""
last mod 4/30/17

We should probably store the IMU data directly on the Reach until we can get the wifi transmitting at a higher rate
The magnetometer parsing code (incomplete) is commented out.
Parsing magnetometer data is a pain, and will probably require calibration...
but we can also just use the gyro/acceleration.
"""
import time
import struct
import numpy as np
from Sensor import Sensor
from connectors import RxConnector

formatstring1 = '<fffffffff' # the format in which the data is being sent
magneticdeclination = 7*np.pi/180 # Austin has a declination ~ 7 degrees East

def parseMessage(message):
    ff = struct.unpack(formatstring1, message)
    formstr2 = ",{:+06.3f}"*3 + ",{:+06.2f}"*3 + ",{:+06.2f}"*3 # format to log
    return formstr2.format(*ff)

#def parseMessage(message):
#    ff = struct.unpack(formatstring1, message)
#    
#    mx,my,mz = ff[6:]
#    netmagfield = mx**2. + my**2. + mz**2. # should be around 60 microT
#    downwardmag = np.arctan2(mz, (mx**2.+my**2.)**.5) # should be around 70deg
#    heading_valid = netmagfield > 10 and netmagfield < 100 and downwardmag > .5
#    angle = np.arctan2(mx,my) + magneticdeclination
#    
#    formstr2 = "{:6.3f}"*3 + "{:6.2f}"*3 + "{:5.2f},{:b}" # format to log
#    return formstr2.format(ff[0],ff[1],ff[2],ff[3],ff[4],ff[5],angle,heading_valid)


class Imu(Sensor):
    def __init__(self, port, filename):
        self.conn = RxConnector(port, 'UDP')
        self.log = open(filename, 'wb')
        Sensor.__init__(self)
        
    def run(self):
        header = 'time,ax,ay,az,gx,gy,gz,mx,my,mz'
        #header = 'time,ax,ay,az,gx,gy,gz,heading,heading_valid'
        self.log.write(header)
        
        loglist = []
        logcounter = 0
        while True:
            news = self.conn.recv(36, 1.1)
            assert len(news) == 36, "IMU connection stopped"
            rcv_time = '\n{:9.3f}'.format(time.time() - self.startTime)
            loglist += [rcv_time + parseMessage(news)]
            logcounter += 1
            if logcounter > 100:
                self.log.write(''.join(loglist))
                loglist = []
                logcounter = 0
            
    def clean(self):
        self.log.close()
        self.conn.__exit__()
