# -*- coding: utf-8 -*-
"""
simple example of this simulator
last mod 3/11/2017
"""

from OwnSim import RoadMap, Simulator

GUI = True
SIMDELAY = 0.3 # (s)

## 4 way, 2 lane intersection
roads = {'1i_0':(0, 95, 92, 95),
         '1i_1':(0, 98.35, 92, 98.35),
         '2i_0':(200, 105, 108, 105),
         '2i_1':(200, 101.65, 108, 101.65),
         '3i_0':(105, 0, 105, 92),
         '3i_1':(101.65, 0, 101.65, 92),
         '4i_0':(95, 200, 95, 108),
         '4i_1':(98.35, 200, 98.35, 108),
         '2o_0':(108, 95, 200, 95),
         '2o_1':(108, 98.35, 200, 98.35),
         '1o_0':(92, 105, 0, 105),
         '1o_1':(92, 101.65, 0, 101.65),
         '4o_0':(105, 108, 105, 200),
         '4o_1':(101.65, 108, 101.65, 200),
         '3o_0':(95, 92, 95, 0),
         '3o_1':(98.35, 92, 98.35, 0)}

# which roads connect to other roads
intersections   =    [['1i_0','3o_0'] , ['1i_0','2o_0'] , ['1i_1','4o_1'] ,
                      ['2i_0','4o_0'] , ['2i_0','1o_0'] , ['2i_1','3o_1'] ,
                      ['3i_0','2o_0'] , ['3i_0','4o_0'] , ['3i_1','1o_1'] ,
                      ['4i_0','1o_0'] , ['4i_0','3o_0'] , ['4i_1','2o_1'] ]

roadMap = RoadMap(roads, intersections)

Sim = Simulator(roadMap, gui = GUI, delay = SIMDELAY)
Sim.createVehicle('mycar', '1i_0', 50.)

Sim.moveVehicle('mycar', '1i_1', 50.)
for k in range(20):
    exited = Sim.moveVehicleAlong('mycar', 5., '4o_1')
    
    carLane,carLanePos,carPos,carAngle = Sim.getVehicleState('mycar')
    print carPos
    
    manual_escape = Sim.updateGUI(allowPause=True)
    if exited or manual_escape:
        break

Sim.end()
