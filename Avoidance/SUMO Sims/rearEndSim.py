# -*- coding: utf-8 -*-
from sumoMethods import Sumo
from optparse import OptionParser
import Controllers
import pandas as pd
import os.path
import time
import random
import collisionCheck

numiter = 2
outputName = 'rearEnd'


VEHsize = (5,2) # meters, length by width
DELTAT = .1
outputFolder = os.path.realpath('Results')
paramFolder = os.path.realpath('Parameters')
configuration = 'emptyhighway'
    
def vehicleSpeed():
    return random.uniform(55,85)*.447

def vehicleAccel():
    return random.uniform(-1,1)

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

ParameterColumns = ['Ego Speed', 'Lead Speed', 'Ego Accel', 'Lead Accel',
                    'Brake Magnitude', 'Brake Time',
                    'Collision Time']
params = None
outputColumns = ['time','vehID','x','y','angle','speed']


## run iteration
iteration=1
while iteration <= options.numiter:
    err = 0
    starttime = time.time()
    Sim = Sumo(configuration, options.gui)
    
    ## now set up parameters and vehicles
    err += Sim.createVehicle('ego','main_0',VEHsize[0])
    egoControl = Controllers.RearEndEgo(vehicleSpeed(), vehicleAccel())
    err += Sim.createVehicle('lead','main_0',50+VEHsize[0])
    leadControl = Controllers.RearEndBrake(vehicleSpeed(), vehicleAccel())
    
    params_iter = pd.DataFrame([[egoControl.speed, leadControl.speed,
                                 egoControl.accel, leadControl.accel,
                                 leadControl.brakeMagnitude, leadControl.brakeTime,
                                 -1.]], columns=ParameterColumns)
    output = None

    ttime = 0
    maxTime = 100 # seconds
    lanelength = Sim.getLaneInfo('main_0')[0]
    
    
    ## run simulation
    while ttime < maxTime:
        # get info on this step
        aa,egoLanePos,egoPos = Sim.getVehicleState('ego')
        aa,leadLanePos,leadPos = Sim.getVehicleState('lead')
        egoControl.update(DELTAT)
        leadControl.update(DELTAT)
        egoSpeed = egoControl.nextStep()
        leadSpeed = leadControl.nextStep()
        
        # save
        egoState = [ttime, 'ego', egoPos[0], egoPos[1], 90., egoSpeed]
        leadState = [ttime, 'lead', leadPos[0], leadPos[1], 90., leadSpeed]
        if output is None:
            output = pd.DataFrame([egoState, leadState])
        else:
            output = output.append([egoState, leadState])
        
        # check for first collision - others too difficult to store..
        if params_iter['Collision Time'].iloc[0] < 0:
            egoObject = pd.Series(egoState[2:5] + list(VEHsize),
                                  index=collisionCheck.collisionVars)
            leadObject = pd.Series(leadState[2:5] + list(VEHsize),
                                   index=collisionCheck.collisionVars)
            if collisionCheck.check(egoObject,leadObject):
                params_iter['Collision Time'].iloc[0] = ttime
        
        # construct next step
        ttime += DELTAT
        if egoSpeed > 0.:    
            err += Sim.moveVehicleAlong('ego', egoSpeed * DELTAT)
        if leadSpeed > 0.:
            err += Sim.moveVehicleAlong('lead', leadSpeed * DELTAT)
            
        # check for ending of simulation
        if lanelength - max(leadLanePos, egoLanePos) < 10:
            # either vehicle is near the end of the road
            break
        if egoLanePos - leadLanePos > 20:
            # ego passed lead a while ago
            break
        if err > 0:
            break
    
    if err == 0:
        Sim.end()
        
        # save iteration information
        outputFile = outputFolder+'/'+options.outputName+str(iteration)+'.csv'
        output.columns = outputColumns
        output.to_csv(outputFile, sep=',',header=True,index=False)
        if params is None:
            params = params_iter
        else:
            params = params.append(params_iter)
        
        # finish step
        print "iteration "+str(iteration)+" : "+str(time.time()-starttime)
        iteration += 1
        time.sleep(.05)
    else:
        print "iteration "+str(iteration)+" failed, retrying"
        
        time.sleep(1.)
    
paramFile = paramFolder+'/'+options.outputName+'_param.csv'
params.to_csv(paramFile, header=True,index=False)