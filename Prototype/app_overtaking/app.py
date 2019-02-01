# -*- coding: utf-8 -*-
"""
last mod 11/26/2018
"""

import numpy as np
from prototype.app_overtaking.audioWarning import audioWarning# as display
import prototype.app_overtaking.options as options

if options.real_vs_simulator:
    from prototype.sensors.dsrcsensor import DsrcSensor as Sensor
else:
    from prototype.app_overtaking.simulation import SimSensor as Sensor
    
    
def projectLat(vector1, vector2): # cc, assumes vector2 is normalized
    return vector1[0]*vector2[1] - vector1[1]*vector2[0]
    
def projectLong(vector1, vector2):
    return vector1[0]*vector2[0] + vector1[1]*vector2[1]
    
def solveQuadratic(x,v,a): # a/2*t**2 + v*t = x
    return (np.sqrt(v*v + a*x*2) - v)/a

# assume you are ok with the vehicle being 1.5 meters (at minimum) from either other vehicle
safe_gap = 1.5
safe_gap = options.vehicle_len + safe_gap*2

class Display():
    def __init__(self): pass
    def __enter__(self): return self
    def __exit__(self,a,b,c): pass
    def get(self): pass

with Sensor(options.DSRC_PORT) as dsrc, Display() as display:
    while True:
        you_return, alt_cars = dsrc.get()
        
        you_pos, you_vel = you_return
        you_speed = np.hypot(you_vel[0], you_vel[1])
        you_vel /= you_speed
        
        has_oncoming = False
        nearest_oncoming_car_pos = 10000
        nearest_oncoming_car_speed = 0
        has_lead = False
        nearest_lead_car_pos = 10000
        nearest_lead_car_speed = 0
        for alt_pos, alt_vel, alt_ID in alt_cars:
            pos_lat = projectLat(alt_pos - you_pos, you_vel)
            pos_long = projectLong(alt_pos - you_pos, you_vel)
            alt_speed = np.hypot(alt_vel[0], alt_vel[1])
            vel_long = projectLong(alt_vel, you_vel)
            vel_lat = projectLat(alt_vel, you_vel)
            is_oncoming_pos = pos_lat > -6 and pos_lat < 2 and pos_long > 20
            is_oncoming_vel = alt_speed < 3 or (-vel_long > abs(vel_lat) * 2)
            if is_oncoming_pos and is_oncoming_vel:
                # this is potentially an oncoming vehicle
                if not has_oncoming or pos_long < nearest_oncoming_car_pos:
                    nearest_oncoming_car_pos = pos_long
                    nearest_oncoming_car_speed = -vel_long
                    has_oncoming = True
            is_lead_pos = pos_lat > -2 and pos_lat < 6
            is_lead_pos &= pos_long > 0 and pos_long < 80
            is_lead_vel = alt_speed < 3 or (vel_long > abs(vel_lat) * 2)
            if is_lead_pos and is_lead_vel:
                if not has_lead or pos_long < nearest_lead_car_pos:
                    nearest_lead_car_pos = pos_long
                    nearest_lead_car_speed = vel_long
                    has_lead = True
                    
        if not has_lead:
            print("no lead to overtake detected")
            # invalid display
            continue
        if not has_oncoming:
            print("no oncoming vehicle detected")
            # safe display
            continue
        
        if nearest_lead_car_pos < 0: nearest_lead_car_pos = 0
        
        # determine oncoming-lead crossing time
        onco_lead_speed = nearest_oncoming_car_speed + nearest_lead_car_speed
        onco_lead_pos = nearest_oncoming_car_pos - nearest_lead_car_pos - safe_gap
        onco_lead_accel = .2 # for both vehicles
        crossing_time = solveQuadratic(onco_lead_pos, onco_lead_speed, onco_lead_accel)
        crossing_time = max(crossing_time, 0)
        # determine distance from current oncoming position at crossing time
        crossing_pos = nearest_lead_car_pos + nearest_lead_car_speed * crossing_time
        crossing_pos += .05*crossing_time*crossing_time + safe_gap
        # determine the accleration this car needs for successful overtaking
        overtaking_accel = (crossing_pos - you_speed*crossing_time)*2./crossing_time/crossing_time
        
        if crossing_time < 15 and overtaking_accel > options.reasonable_overtaking_accel:
            print("UNSAFE oncoming car in {:.0f} seconds".format(crossing_time))
            # warn display
        elif overtaking_accel > options.reasonable_overtaking_accel:
            # invalid display
            print("overtake not recommended")
        else:
            print("oncoming car in {:.0f} seconds".format(crossing_time))
            # safe display
