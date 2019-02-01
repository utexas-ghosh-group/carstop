# -*- coding: utf-8 -*-
"""
last mod 11/21/18
This file contains all the variables you (should) need to understand or modify
"""

# if True uses real DSRC input
# if False uses simulation
real_vs_simulator = False

# the port number that the DSRC unit is talking to
# if the simulation is being run, this number is not important
port = 9000

# road files are explained in more detail in road.py
# you have to make one for any location where this code is being used, and set
# the filename here appropriately
road_file = 'prototype/app_highwaymerge/roads/road_innovation.txt'

# the following parameters that can be adjusted for each vehicle or application

# how far back of car is from gps antenna - where position is measured (meters)
you_rear_length = 3.

# if car is currently merging, check for collisions within this time (seconds)
time_merging_warn = .5

# if car is waiting to merge, check for collisions within this time (seconds)
time_waiting_warn = 2.

# zone within which any oncoming vehicle should cause a warning (meters)
# the idea is that within this range, drivers can judge for themselves
# and are possibly worried if a car comes within this range w/o a warning
warning_zone = 90.

# the highest an oncoming car is assumed to accelerate (m/s^2)
# car's can reach around 9, but this is highly unusual outside of a race
max_accel = 3. 

# if lower than this car is assumed to be stopped (m/s)
stationary_speed = .5
