# -*- coding: utf-8 -*-
"""
last mod 11/23/18
This file contains all the variables you (should) need to understand or modify
"""

# if True uses real DSRC input
# if False uses simulation
real_vs_simulator = False

# the name of the DSRC (maybe not important?)
DSRC_IP = '168.192.115.37'

# the port number that the DSRC unit is talking to
# if the simulation is being run, this number is not important
DSRC_PORT = 9000

# the length of this vehicle (meters)
vehicle_len = 5.

# the highest acceleration the human driver can be assumed to provide (m/s^2)
reasonable_overtaking_accel = 1.5