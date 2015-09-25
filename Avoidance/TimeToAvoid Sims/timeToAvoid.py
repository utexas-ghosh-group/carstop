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

# avoidance motion parameters
max_accel = 1.
max_decel = 1.
max_left = 1.
max_right = 1.

emergencyMoves = ((max_accel, 0), (max_accel, max_left), (max_accel, -max_right),
                  (max_decel, 0), (max_decel, max_left), (max_decel, -max_right))
# per-simulation parameters
ego_v = 1.
ego_a = 1.
ego_wheel = .3

alter_v = 1.
alter_a = 1.
alter_head = 1. # in radians
alter_wheel = .3

# for now, just let starting position be center
# this should change...


    

# start at center facing East
ego = Vehicle(0, 0, ego_v, 0, ego_a, ego_wheel)
vsize = (VEHlength, VEHwidth)

# to initiate alter vehicle, need to find the exact beginning of collision
# this is computationally intractable
# instead start with some randomness and search for time of first collision
alter = None
emergEscape = 0
while alter == None and emergEscape < 1000:
    xcenter = ego.center()
    temp_alter = Vehicle(xcenter[0] + uniform(-VEHlength, VEHlength),
                             xcenter[1] + uniform(-VEHlength, VEHlength),
                             alter_v, alter_head, alter_a, alter_wheel)
    # move backwards until you find first collision time
    timesBackwards = np.arange(-DELT, -5, -DELT)
    egoPositions = ego.moveCA(timesBackwards)
    alterPositions = temp_alter.moveCA(timesBackwards)
    collisions = collisionCheck.check(egoPositions,alterPositions,vsize,vsize)
    aa = next((i for i in reversed(range(len(collisions))) if collisions[i]), None)
    if not aa == None:  # never was a collision
        ego.moveSelf(timesBackwards[aa])
        alter = temp_alter
        alter.moveSelf(timesBackwards[aa])
    emergEscape = emergEscape + 1

## now can use emergency movements
ttA = 0
avoided = False
while not avoided and ttA < 20:
    ttA = ttA + DELT
    past_ego = ego.copy()
    past_ego.moveSelf(-ttA)
    past_alter = alter.copy()
    past_alter.moveSelf(-ttA)
    
    timesAhead = np.arange(DELT, ttA+2, DELT) # going a second past the original collision
    alterPositions = past_alter.moveCA(timesAhead)
    
    for movement in emergencyMoves:
        past_ego.accel = movement[0]
        past_ego.wheel = movement[1]
        egoPositions = past_ego.moveCA(timesAhead)
        
        collisions = collisionCheck.check(egoPositions,alterPositions,vsize,vsize)
        if not np.any(collisions): # this movement was entirely safe
            avoided = True
            break