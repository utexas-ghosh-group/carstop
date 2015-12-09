# -*- coding: utf-8 -*-
from sumoMethods import Sumo
from optparse import OptionParser
import Controllers
import pandas as pd
import os.path
import time

configuration = 'emptyhighway'
numiter = 50
outputName = 'rearEnd'


VEHsize = (5,2) # meters, length by width
DELTAT = .1
outputFolder = os.path.realpath('Results')


optParser = OptionParser()
optParser.add_option("-g", "--gui", action="store_true", dest="gui",
                     default=False, help="run with GUI")
optParser.add_option("-i", "--iter", type="int", dest="numiter",
                     default=numiter, help="number of times to run")
optParser.add_option("-o", "--output", type="string", dest="outputName",
                     default=outputName, help="name to save output csv's as")
#optParser.add_option("-c", "--config", type="string", dest="configuration",
#                     default=configuration, help="SUMO config to use")
#optParser.add_option("-v", "--verbose", action="store_true", dest="verbose",
#                     default=False, help="tell me what you are doing")
(options, args) = optParser.parse_args()


## run SUMO
for iteration in range(options.numiter):
    Sim = Sumo(configuration, options.gui)  
    
    outputColumns = ['time','vehID','x','y','angle','speed']
    output = None
    
    ## now control the simulation
    controllers = {}
    Sim.createVehicle('ego','main_0',VEHsize[0])
    controllers['ego'] = Controllers.RearEndEgo(25)
    Sim.createVehicle('lead','main_0',40)
    controllers['lead'] = Controllers.RearEndBrake(25)
    lanelength,aa,bb = Sim.getLaneInfo('main_0')
    
    
    ttime = 0
    maxTime = 100 # seconds
    starttime = time.time()
    while ttime < maxTime:
        # handle this step
        aa,egoLanePos,egoPos = Sim.getVehicleState('ego')
        aa,leadLanePos,leadPos = Sim.getVehicleState('lead')
        controllers['ego'].update(DELTAT)
        controllers['lead'].update(DELTAT)
        egoSpeed = controllers['ego'].nextStep()
        leadSpeed = controllers['lead'].nextStep()
        
        # save
        egoState = [ttime, 'ego', egoPos[0], egoPos[1], 90., egoSpeed]
        leadState = [ttime, 'lead', leadPos[0], leadPos[1], 90., leadSpeed]
        if output is None:
            output = pd.DataFrame([egoState, leadState])
        else:
            output = output.append([egoState, leadState])
        
        # check for ending of simulation
        if lanelength - max(leadLanePos, egoLanePos) - VEHsize[0] < 5:
            # either vehicle is near the end of the road
            break
        if egoLanePos - leadLanePos > 20:
            # ego passed lead a while ago
            break
        
        # construct next step
        ttime += DELTAT
        Sim.moveVehicleAlong('ego', egoSpeed * DELTAT)
        Sim.moveVehicleAlong('lead', leadSpeed * DELTAT)
  
    endtime = time.time()
    print "iteration "+str(iteration+1)+" loop "+str(endtime-starttime)
    Sim.end()
    print "      wait "+str(time.time()-endtime)
    
    outputFile = outputFolder+'/'+options.outputName+str(iteration+1)+'.csv'
    output.columns = outputColumns
    output.to_csv(outputFile, sep=',',header=True,index=False)