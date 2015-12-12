# -*- coding: utf-8 -*-
"""
Created on Fri Sep 25 18:26:24 2015

@author: motro
"""
import random as rnd
from numpy import sin, pi


#helper methods
def alph2om(wheelAngle, velocity):
    # converts wheel angle to angular velocity (0-acceleration approx.)
    return velocity * sin(wheelAngle) / 4

def simpleEmergencyMoves(max_accel, max_decel, max_left, max_right):
    # moves are either max accel or decel, with max turns or no turn
    return ((max_accel, 0), (max_accel, max_left), (max_accel, -max_right),
            (-max_decel, 0), (-max_decel, max_left), (-max_decel, -max_right))


class InitializeParams:
    """
    ego is vehicle with avoidance capability
        starts at (0,0) facing +x direction
    alter is other vehicle, starts in some collision position with ego
    
    parameters that must be specified:    
    ego_velocity - in m/s
    ego_accel - in m/s
    ego_angVel - in rad/s
    
    alter_heading - the angle Alter makes at time of collision
        ex: pi/2 means that Alter is moving +y (crashes into ego's right)
    alter_velocity
    alter_accel
    alter_angVel
    
    emergencyMoves - list of (acceleration, angularVelocity) tuples
    """
    def __init__(self, paramName):        
        if paramName == 'Free':
            self.initFree()
        elif paramName == 'Rural Road':
            self.initRuralRoad()
        elif paramName == 'Rear End':
            self.initRearEnd()
        else:
            print 'Scenario not recognized'
        
    def outputList(self):
        # returns parameters, currently not used
        return [self.ego_velocity, self.ego_accel, self.ego_angVel,
                self.alter_velocity, self.alter_accel, self.alter_heading, 
                self.alter_angVel, self.emergencyMoves]
        
        
    def initFree(self):
        self.ego_velocity = rnd.uniform(15,30)
        self.ego_accel = rnd.uniform(-1,1)
        typAngVel = 20 * pi/180.
        #typAngVel = alph2om(pi/10 , self.ego_velocity)
        self.ego_angVel = rnd.uniform(-typAngVel, typAngVel)
        
        self.alter_heading = rnd.uniform(-pi,pi)
        self.alter_velocity = rnd.uniform(15,30)
        self.alter_accel = rnd.uniform(-1,1)
        #typAngVel = alph2om(pi/10 , self.alter_velocity)
        self.alter_angVel = rnd.uniform(-typAngVel, typAngVel)
        
        # 20 deg/s is max the occurs, 40 is max for normal cars without flipping, etc.
        bigAngVel = 40 * pi/180.
        #bigAngVel = alph2om(pi/6 , self.ego_velocity)
        self.emergencyMoves = simpleEmergencyMoves(2,2,bigAngVel,bigAngVel)
        
        
    def initRuralRoad(self):
        self.ego_velocity = rnd.uniform(20,40)
        self.ego_accel = rnd.uniform(-1,1)
        typAngVel = 10 * pi/180.
        self.ego_angVel = rnd.uniform(-typAngVel, typAngVel)
        
        self.alter_heading = rnd.uniform(pi-.5, pi)*rnd.choice((-1,1)) # head-on collisions
        self.alter_velocity = rnd.uniform(20,40)
        self.alter_accel = rnd.uniform(-1,1)
        self.alter_angVel = rnd.uniform(-typAngVel, typAngVel)
        
        bigAngVel = 20 * pi/180. # can't turn too much
        self.emergencyMoves = simpleEmergencyMoves(2,3,bigAngVel,bigAngVel)
        
        
    def initRearEnd(self):
        self.ego_velocity = rnd.uniform(55,85)*.447
        self.ego_accel = rnd.uniform(-.8,.8)
        typAngVel = 2 * pi/180.
        #typAngVel = alph2om(pi/10 , self.ego_velocity)
        self.ego_angVel = rnd.uniform(-typAngVel, typAngVel)
        
        self.alter_heading = 0.#rnd.uniform(-pi,pi)
        self.alter_accel = rnd.uniform(-6,-8)
        self.alter_velocity = max(rnd.uniform(55,85)*.447 +
                                self.alter_accel*rnd.uniform(.5,4), 0.)
        # this is speed after crash, assume its been braking for a while
        #typAngVel = alph2om(pi/10 , self.alter_velocity)
        self.alter_angVel = rnd.uniform(-typAngVel, typAngVel)
        
        # 20 deg/s is max the occurs, 40 is max for normal cars without flipping, etc.
        bigAngVel = 10 * pi/180.
        #bigAngVel = alph2om(pi/6 , self.ego_velocity)
        self.emergencyMoves = simpleEmergencyMoves(2,8,bigAngVel,bigAngVel)