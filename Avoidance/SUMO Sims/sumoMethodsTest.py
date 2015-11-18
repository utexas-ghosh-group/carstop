# -*- coding: utf-8 -*-
"""
Created on Tue Nov 17 20:13:37 2015

@author: motro
"""

from sumoMethods import Sumo

configuration = 'emptyhighway'

sim = Sumo(configuration)

sim.createVehicle('alone','main_0')

sim.moveVehicleAlong('alone',30)

[lane,lanex,xy] = sim.getVehicleState('alone')

sim.end()