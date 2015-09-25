# -*- coding: utf-8 -*-
"""
Created on Wed Sep 23 16:54:44 2015

@author: motro
"""
import numpy as np
from random import uniform
import collisionCheck
from simVehicle import Vehicle, VEHlength, VEHwidth

# unchanging parameters
DELT = 0.1   # resolution of time measures

max_accel = 1.
max_decel = 1.
max_left = 1.
max_right = 1.
emergencyMoves = ((max_accel, 0), (max_accel, max_left), (max_accel, -max_right),
                  (-max_decel, 0), (-max_decel, max_left), (-max_decel, -max_right))    

def timeToAvoid(ego_v, ego_a, ego_wheel, alter_v, alter_a, alter_head,
                alter_wheel, emergencyMoves):
    # inputs are ints, except 
    # emergencyMoves = [... (accel, wheel), ...]
    # output = shortest time at which collision can be avoided
    # outputs None if time can't be calculated (no collision possible)
    """    
    # per-simulation parameters
    ego_v = 1.
    ego_a = 1.
    ego_wheel = .3
    
    alter_v = 1.
    alter_a = 1.
    alter_head = 1. # in radians
    alter_wheel = .3
    
    max_accel = 1.
    max_decel = 1.
    max_left = 1.
    max_right = 1.
    emergencyMoves = ((max_accel, 0), (max_accel, max_left), (max_accel, -max_right),
                      (-max_decel, 0), (-max_decel, max_left), (-max_decel, -max_right))  
    """    
    
    # start at center facing East
    ego = Vehicle(0, 0, ego_v, 0, ego_a, ego_wheel)
    vsize = (VEHlength, VEHwidth)
    
    # to initiate alter vehicle, need to find the exact beginning of collision
    # this is computationally intractable
    # instead start with some randomness and search for time of first collision
    xcenter = ego.center()    
    alter = None
    emergEscape = 0
    while alter == None and emergEscape < 1000:
        temp_alter = Vehicle(xcenter[0] + uniform(-VEHlength, VEHlength),
                                 xcenter[1] + uniform(-VEHlength, VEHlength),
                                 alter_v, alter_head, alter_a, alter_wheel)
        # check recent past, find first moment of collision
        timesBackwards = np.arange(-5, 0, DELT)
        egoPositions = ego.moveCA(timesBackwards)
        alterPositions = temp_alter.moveCA(timesBackwards)
        collisions = collisionCheck.check(egoPositions,alterPositions,vsize,vsize)
        aa = next((i for i in range(len(collisions)) if collisions[i]), None)
        if not aa == None:  # never was a collision
            ego.moveSelf(timesBackwards[aa])
            alter = temp_alter
            alter.moveSelf(timesBackwards[aa])
        emergEscape = emergEscape + 1
    if emergEscape == 1000:
        return None
    
    ## now can use emergency movements
    ttA = 0
    avoided = False
    while not avoided and ttA < 20:
        ttA = ttA + DELT
        past_ego = ego.copy()
        past_ego.moveSelf(-ttA)
        past_alter = alter.copy()
        past_alter.moveSelf(-ttA)
        
        # check up to 2 seconds past the original collision
        timesAhead = np.arange(DELT, ttA+2, DELT)
        alterPositions = past_alter.moveCA(timesAhead)
        
        for movement in emergencyMoves:
            past_ego.accel = movement[0]
            past_ego.wheel = movement[1]
            egoPositions = past_ego.moveCA(timesAhead)
            
            collisions = collisionCheck.check(egoPositions,alterPositions,vsize,vsize)
            if not np.any(collisions): # this movement was entirely safe
                avoided = True
                break
    if not avoided:
        return None

    angleDifference = ego.head - alter.head
    if angleDifference < -np.pi:
        angleDifference = angleDifference + 2*np.pi
    elif angleDifference > np.pi:
        angleDifference = angleDifference - 2*np.pi
    return [ego.speed, ego.accel, ego.wheel, angleDifference,
              alter.speed, alter.accel, alter.wheel, ttA]