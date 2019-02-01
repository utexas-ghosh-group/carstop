#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 6/13/18
This version has an extra display w/ vehicle positions

input: readSensor function
provide coordinates of self and other objects at each timestep
readSensor should block while waiting for next sensor input
           and raise Exception if there is no input in time
if using multiple sensors, need a helper file that combines their output

output: startDisplay, stopDisplay, changeDisplay functions
display runs in a different process, possibly different device
provide single-integer messages
need a helper file to handle actual communication
    
helpers:
global2Road = gets meters along road wrt some origin point
              also gives lane number, or None if point is not on road
              more details in current road.py
Control Panel\System and Security\Windows Defender Firewall\Customize Settings
"""

import prototype.app_highwaymerge.options as opt
from multiprocessing import freeze_support
if opt.real_vs_simulator:
    from prototype.sensors.dsrcsensor import startSensor, readSensor, stopSensor
else:
    from prototype.app_highwaymerge.simulation import startSensor, readSensor, stopSensor
from prototype.app_highwaymerge.road import global2Road
from prototype.app_highwaymerge.fulldisplay import Display

# display commands
display_new_car = 0
display_wait_warning = 1
display_act_warning = 2
display_safe = 3
display_close = 4

# parameters that can be adjusted for each vehicle or application
merging_accel_dist = opt.time_merging_warn**2 * opt.max_accel/2
waiting_accel_dist = opt.time_waiting_warn**2 * opt.max_accel/2


# these are classes so that they can use history-based decisions
# but for now, very simple decisions
# decides whether you should start merge warning system
class MergeAreaChecker():
    def __init__(self):
        self.ck = False
        
    def check(self, road_pos, lane, speed):
        if self.ck: return True
        if lane >= 0: ### new
            if road_pos < opt.warning_zone:
                print("not enough road mapped to warn in this area!")
            else:
                print("merging at pos {:.0f}, lane {:d}".format(road_pos, lane))
                self.ck = True
        return self.ck

class StartedMergeChecker():
    def __init__(self):
        self.ck = False
        
    def check(self, road_pos, lane, speed):
        if self.ck: return True
        if speed > .75:
            self.ck = True    
        return self.ck
            
class CompletedMergeChecker():
    def __init__(self):
        self.ck = False
        self.seems_to_be_moving = 0
        
    def check(self, road_pos, lane, speed):
        if self.ck: return True
        if lane == 1 and speed > 3:
            self.seems_to_be_moving += 1
        if self.seems_to_be_moving >= 3:
            self.ck = True
        return self.ck


if __name__ == '__main__':
    freeze_support() # this is needed for using GUI in Windows
    try:
        mergeareachecker = MergeAreaChecker()
        startedmergechecker = StartedMergeChecker()
        completedmergechecker = CompletedMergeChecker()
        in_zone_IDs = set()
        started_display = False
        startSensor()
        
        while True:
    
            you_return, alt_cars = readSensor()
            
            you_coord, you_speed = you_return
            you_road_pos, you_lane = global2Road(you_coord)
            
            if not started_display:
                if not mergeareachecker.check(you_road_pos, you_lane, you_speed):
                    continue
                print("in merge area!")
                display = Display(you_coord)
                started_display = True

            # maintain a list of relevant cars
            # current criterion: on the road and not past you yet
            relevant_alt_cars = []
            for alt_coord, alt_speed, alt_ID in alt_cars:
                alt_road_pos, alt_lane = global2Road(alt_coord)
                relevant = not (alt_lane is None) # and alt_speed < opt.stationary_speed)
                relevant &= alt_road_pos <= you_road_pos
                if relevant:
                    relevant_alt_cars.append((alt_road_pos, alt_lane, alt_speed, alt_ID))
                
            
            if startedmergechecker.check(you_road_pos, you_lane, you_speed):
                if completedmergechecker.check(you_road_pos, you_lane, you_speed):
                    break
            
                # vehicles projected to be beyond a certain distance cause warning
                # no maximum distance - vehicles continue to cause warning until
                # they pass you completely
                min_safe_pos = you_road_pos - opt.you_rear_length
                safe = True
                for alt_road_pos, alt_lane, alt_speed, alt_ID in relevant_alt_cars:
                    max_alt_pos = alt_road_pos + opt.time_merging_warn * alt_speed +\
                                                merging_accel_dist
                    if max_alt_pos > min_safe_pos:
                        safe = False
                        
                if safe:
                    display.changeDisplay(display_safe, you_coord, relevant_alt_cars)
                if not safe: # collision considered
                    display.changeDisplay(display_act_warning, you_coord, relevant_alt_cars)
                    
            else:
                
                min_safe_pos = you_road_pos - opt.warning_zone
                new_zone_IDs = set()
                for alt_road_pos, alt_lane, alt_speed, alt_ID in relevant_alt_cars:
                    max_alt_pos = alt_road_pos + opt.time_waiting_warn * alt_speed +\
                                    waiting_accel_dist
                    if max_alt_pos > min_safe_pos:
                        new_zone_IDs.add(alt_ID)
                        if not (alt_ID in in_zone_IDs):
                            display.changeDisplay(display_new_car, you_coord, relevant_alt_cars)
                if len(new_zone_IDs) == 0:
                    display.changeDisplay(display_safe, you_coord, relevant_alt_cars)
                else:
                    display.changeDisplay(display_wait_warning, you_coord, relevant_alt_cars)
                in_zone_IDs = new_zone_IDs
    
    finally:
        stopSensor()
        if started_display:
            display.stopDisplay()
