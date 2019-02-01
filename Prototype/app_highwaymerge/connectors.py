#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 4/21/17
"""
import socket, time, select

class RxConnector():
    def __init__(self, port, protocol='TCP'):
        assert protocol in ('TCP','UDP'), "protocol = 'TCP' or 'UDP'"
        self.protocol = protocol
        if protocol == 'TCP':
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.bind(('',port))
            s.listen(1)
            print("Rx waiting...")
            conn,addr = s.accept()
            self.server_sock = s
            self.s = conn
            print("Rx receiving")
            self.firsttime = False
        else:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.s.bind(('',port))
            self.firsttime = True
            print("Rx waiting...")
	
    def recv(self, size, maxwaittime = 0.1):
        if self.firsttime: # first receive for UDP, wait for a while
            readable,a,b = select.select([self.s], [], [], 30.)
            if len(readable) == 0:
                print("timed out without ever receiving a message")
                return ''
            print("Rx receiving")
            self.firsttime = False
        elif self.protocol == 'UDP':
            readable,a,b = select.select([self.s], [], [], maxwaittime)
            if len(readable)==0:
                print("Rx timed out")
                return ''
        return self.s.recv(size)
    
    def ack(self):
        if self.protocol == 'TCP':
            self.s.sendall("ack")
        else:
            print("UDP ack not supported yet")
    
    def __enter__(self):
        return self
    def __exit__(self, errtype=None, errval=None, traceback=None):
        self.s.close()
        if self.protocol=='TCP':
            self.server_sock.close()

        
class TxConnector():
    def __init__(self, ip, port, protocol = 'TCP'):
        assert protocol in ('TCP','UDP'), "protocol = 'TCP' or 'UDP'"
        self.protocol = protocol
        if protocol == 'TCP':
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            print("Tx connecting...")
            self.s.connect((ip, port))
            print("Tx connected")
        else:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.ip = ip
            self.port = port
        self.exitmessage = None
        
    def send(self, msg):
        if self.protocol == 'TCP':
            self.s.sendall(msg)
        else:
            self.s.sendto(msg, (self.ip, self.port))
        
    def setExitMessage(self, msg):
        self.exitmessage = msg
        
    def ack(self, maxwaittime = 0.5):
        readable,a,b = select.select([self.s], [], [], maxwaittime)
        if len(readable)==0:
            print("Tx ack timed out")
            return ''
        got = self.s.recv(3)
        assert got=="ack" , "Tx ack = {}".format(got)
        
    def __enter__(self):
        return self
    def __exit__(self, errtype, errval, traceback):
        if self.exitmessage:
            self.send(self.exitmessage)
            time.sleep(.1)
        self.s.close()
        
        
""" for testing, does nothing """
class NullConnector():
    def __init__(self, *args): pass
    def send(self, *args): pass
    def recv(self, *args): pass
    def ack(self, *args): pass
    def setExitMessage(self, *args): pass
    def __enter__(self): pass
    def __exit__(self, errtype, errval, traceback): pass