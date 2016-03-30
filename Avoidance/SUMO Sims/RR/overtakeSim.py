# -*- coding: utf-8 -*-
"""
Overtaking scenario
3/28/16
"""

simulatorFolder = '../../../simulator'
outputFolder = '../Results/RR'
parametersFile = 'simParameters.csv'
numsims = 5000


import sys,os
sys.path.append(os.path.realpath(simulatorFolder))
from OwnSim import RoadMap
from OwnSim import Simulator

from subprocess import call as terminalCall
if os.name == 'posix': # unix
    terminalCall(['mkdir','-p',os.path.realpath(outputFolder)])
else:
    terminalCall(['mkdir',os.path.realpath(outputFolder)], shell=True)

import pandas as pd
params = pd.read_csv(parametersFile)

# setup
roads = {'main':(0.,2.,1300.,2.), 'oncoming':(1300.,5.2,0.,5.2),
         'overtaking':(0.,5.2,1300.,5.2)}
roadMap = RoadMap(roads, [])

class Output:
    def __init__(self, colnames):
        self.df = None
        self.colnames = colnames
    def add(self, newrow):
        if self.df is None:
            self.df = pd.DataFrame([newrow])
        else:
            self.df = self.df.append([newrow])
    def write(self, fileName, restart=False):
        self.df.columns = self.colnames
        self.df.to_csv(fileName, sep=',', header=True, index=False)
        if restart:
            self.df = None
output = Output(['time','vehID','x','y','angle','speed','Acceleration'])

def calcHeadway(distance, speed, accel):
    if distance < 0:
        return -1
    if speed < 0 and accel < 0:
        return 100
    if accel == 0:
        return distance / speed
    discrim = (speed**2 + 2*accel*distance)
    if discrim < 0:
        return 100
    return (discrim**.5 - speed) / accel

DT = 0.1

# simulations
for simIndex in range(numsims):
    parameters = params.iloc[simIndex]
    sim = Simulator(roadMap, gui = False, delay=0., waitOnStart = True)
    sim.createVehicle('Passing','main',parameters['Passing Position'])
    sim.createVehicle('Lead','main',parameters['Lead Position'])
    sim.createVehicle('Oncoming','oncoming',parameters['Oncoming Position'])
    
    time = 0
    continueSim = True
    overtakeNotComplete = True
    vPass = parameters['Passing Speed']
    vLead = parameters['Lead Speed']
    vOncom = parameters['Oncoming Speed']
    accPass = parameters['Passing Acceleration']
    accLead = parameters['Lead Acceleration']
    accOncom = parameters['Oncoming Acceleration']

    while continueSim:
            
        currentAccPass = 0.
        if time > parameters['tpr']:
            currentAccPass = accPass
        
        # save current information        
        passlane, lp, passPosition, passAngle = sim.getVehicleState('Passing')
        lane, lp, leadPosition, leadAngle = sim.getVehicleState('Lead')
        lane, lp, oncomPosition, oncomAngle = sim.getVehicleState('Oncoming')
        output.add([time, 'Passing', passPosition[0], passPosition[1],
                        passAngle, vPass, currentAccPass])
        output.add([time, 'Lead', leadPosition[0], leadPosition[1],
                        leadAngle, vLead, accLead])
        output.add([time, 'Oncoming', oncomPosition[0], oncomPosition[1],
                        oncomAngle, vOncom, accOncom])
        
        # switch lanes
        leadHeadway = calcHeadway(passPosition[0]-leadPosition[0]-5.8, vLead,
                                  accLead)
        if time > parameters['tpr'] and passlane=='main' and overtakeNotComplete:
            sim.moveVehicle('Passing','overtaking')
        if passlane=='overtaking' and leadHeadway >= 1.:
                sim.moveVehicle('Passing','main')
                overtakeNotComplete = False
        # move vehicles
        vehicleLeft = 0
        vehicleLeft += sim.moveVehicleAlong('Passing', vPass*DT+currentAccPass/2*DT*DT)
        vehicleLeft += sim.moveVehicleAlong('Lead', vLead*DT+accLead/2*DT*DT)
        vehicleLeft += sim.moveVehicleAlong('Oncoming', vOncom*DT+accOncom/2*DT*DT)
        vPass += currentAccPass*DT
        vLead += accLead*DT
        vOncom += accOncom*DT
        
        time += DT
        sim.updateGUI()
        continueSim &= vehicleLeft == 0 # at least one vehicle exits sim
        continueSim &= time < 30. # shouldn't take this long
        continueSim &= (time <= parameters['tpr']+.1 or leadHeadway <= 1.1)
        #continueSim &= leadHeadway <= 1.4 # for better GUI
    
    sim.end()
    outputFile = os.path.realpath(outputFolder+'/'+str(simIndex+1)+'.csv')
    output.write(outputFile, restart=True)
    print simIndex