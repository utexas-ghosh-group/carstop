# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 20:13:37 2015

@author: motro
"""

from sumoMethods import Sumo

configuration = 'emptyhighway'

sim = Sumo(configuration, gui=True)

sim.createVehicle('a','main_0',40)
[lane,lanex,xy] = sim.getVehicleState('a')
print 'a, 0: '+str(xy[0])

sim.createVehicle('b','main_0')
[lane,lanex,xy] = sim.getVehicleState('b')
print 'b, 0: '+str(xy[0])

sim.moveVehicleAlong('b',38)
[lane,lanex,xy] = sim.getVehicleState('a')
print 'a, 1: '+str(xy[0])
[lane,lanex,xy] = sim.getVehicleState('b')
print 'b, 1: '+str(xy[0])

sim.end()