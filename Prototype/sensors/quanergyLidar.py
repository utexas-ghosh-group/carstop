#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 4/3/17
"""
# firing rate : 53828
# packet rate: ~1076
# ~ 7 MBps
import socket
from multiprocessing import Process, Manager
import time
import struct

preamble = ''.join((chr(int(byte, 16)) for byte in ['75','bd','7e','97']))
preamble += struct.pack(">L", 6632)
angle_conversion = 3.1415927 / 10400.
packedstructure = ">H"+"L"*8+"B"*8

def readFromLIDAR(msg, referenceTime, readraw, readhalf):
    assert msg[:8] == preamble, "LIDAR status packet "+','.join((str(ord(char)) for char in msg[:8]))
     
    timestr = '{:10.4f}\n'.format(time.time() - referenceTime)
    if readraw:
        return timestr + msg[8:]
        
    firingidxlist = (firing*132 for firing in range(50))
    firinglist = (msg[f+20:f+22]+msg[f+24:f+56]+msg[f+120:f+128]
                        for f in firingidxlist)
    if readhalf:
        return timestr + '\n'.join(firinglist)
     
    firingdata = (struct.unpack(packedstructure, f) for f in firinglist)
    return [timestr] + list(firingdata)


class LIDAR(Process):
    def __init__(self, IP, queue, readtype = 'raw'):
        Process.__init__(self)
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.IP = IP
        self.queue = queue
        self.readraw = readtype == 'raw'
        self.readhalf = readtype == 'half'
        
    def start(self, referenceTime):
        self.startTime = referenceTime
        Process.start(self)
        
    def run(self):
        msg = ''
        while True:
            while len(msg) < 6632:
                msg += self.socket.recv(4096)
            self.queue.put(readFromLIDAR(msg[:6632], self.startTime,
                                         self.readraw, self.readhalf))
            msg = msg[6632:]
            
    def terminate(self):
        self.socket.close()
        super(LIDAR, self).terminate()
        
    def __enter__(self):
        print "connecting to LIDAR..."
        self.socket.connect((self.IP, 4141))
        print "connected to LIDAR"
	return self
    def __exit__(self, errtype, errval, traceback):
        print errtype
        print errval
        self.terminate()
        
        
# test by saving to file for 2 seconds
if __name__ == '__main__':
    queue = Manager().Queue()
    with LIDAR('129.116.100.217', queue, readtype='half') as lidar:
        time.sleep(10.)
        lidar.start(time.time())
        time.sleep(40.)
    # should terminate now
    
    with open('lidar.dat','w') as datafile:
        while not queue.empty():
            ff = queue.get(timeout=.1)
            datafile.write(str(ff) + '\n')
