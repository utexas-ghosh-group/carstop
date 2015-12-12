# -*- coding: utf-8 -*-
from sumoMethods import Sumo
from optparse import OptionParser
import pandas as pd
import os
import time
from random import uniform
import collisionCheck
from subprocess import call

numiter = 5
outputName = 'inter'


VEHsize = (5,2) # meters, length by width
DELTAT = .1
outputFolder = os.path.realpath('Results')
paramFolder = os.path.realpath('Parameters')
configuration = 'emptyInter2l'
    

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

paramFile = paramFolder+'/'+options.outputName+'_param.csv'
ParameterColumns = ['Ego Speed', 'left Speed', 'Ego Accel', 'left Accel',
                    'Collision Time','Collision Vehicle']
outputColumns = ['time','vehID','x','y','angle','speed']


# smart restart if file is saved
if 'lastStart.int' in os.listdir(os.path.realpath('.')):
    lastStart = open('lastStart.int')
    iteration = int(lastStart.read())
    lastStart.close()
    params = pd.read_csv(paramFile,header=0)    
else:
    iteration = 1
    params = None

## run iteration
while iteration <= options.numiter:
    err = 0
    starttime = time.time()
    Sim = Sumo(configuration, options.gui)
    
    ## now set up parameters and vehicles
    err += Sim.createVehicle('ego','1i_0',0.)
    egoSpeed = 25.
    err += Sim.createVehicle('left','3i_1',0.)
    leftSpeed = 20.
    
    params_iter = pd.DataFrame([[egoSpeed, leftSpeed, 0., 0., -1., 'na']],
                               columns=ParameterColumns)
    output = None

    ttime = 0
    maxTime = 100 # seconds
    
    ## run simulation
    while ttime < maxTime:
        # get info on this step
        egoLane,egoLanePos,egoPos,egoAngle = Sim.getVehicleState('ego')
        leftLane,leftLanePos,leftPos,leftAngle = Sim.getVehicleState('left')
        
        # save
        egoState = [ttime, 'ego', egoPos[0], egoPos[1], egoAngle, egoSpeed]
        leftState = [ttime, 'left', leftPos[0], leftPos[1], leftAngle, leftSpeed]
        if output is None:
            output = pd.DataFrame([egoState, leftState])
        else:
            output = output.append([egoState, leftState])
        
        # check for first collision - others too difficult to store..
        if params_iter['Collision Time'].iloc[0] < 0:
            egoObject = pd.Series(egoState[2:5] + list(VEHsize),
                                  index=collisionCheck.collisionVars)
            leftObject = pd.Series(leftState[2:5] + list(VEHsize),
                                   index=collisionCheck.collisionVars)
            if collisionCheck.check(egoObject,leftObject):
                params_iter['Collision Time'].iloc[0] = ttime
                params_iter['Collision Vehicle'].iloc[0] = 'left'
        
        # construct next step
        ttime += DELTAT
        egoDist = egoSpeed*DELTAT
        if Sim.getLaneInfo(egoLane)[0] - egoLanePos <= egoDist:
            if egoLane[1]=='i': # ego entering intersection
                if uniform(0,2)<1:
                    err += Sim.moveVehicleAlong('ego',egoDist, '2o_0')
                else:
                    err += Sim.moveVehicleAlong('ego',egoDist, '2o_1')
            elif egoLane[1]=='o': # remove vehicle or exit simulation
                break
            #else: # ego leaving intersection
            #    destLane = Sim.getLaneInfo(egoLane)[2][0]
            #    err += Sim.moveVehicleAlong('ego',egoDist,destLane)
        elif egoDist > 0.:    
            err += Sim.moveVehicleAlong('ego', egoDist)
            
        leftDist = leftSpeed*DELTAT
        if Sim.getLaneInfo(leftLane)[0] - leftLanePos < leftDist:
            if leftLane[1]=='i': # ego entering intersection
                err += Sim.moveVehicleAlong('left',leftDist, '1o_1')
            elif leftLane[1]=='o': # remove or exit sim
                break
            #else:
            #    err += Sim.moveVehicleAlong('left',leftDist, '1o_1')
        elif leftDist > 0.:    
            err += Sim.moveVehicleAlong('left', leftDist)
            
        # check for ending of simulation
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
        print "iteration "+str(iteration)+" failed, SUMO is broken"
        print "restart python and rerun, will continue from iter. "+str(iteration)
        lastStart = open('lastStart.int','w')
        lastStart.write(str(iteration))
        lastStart.close()
        break
    
params.to_csv(paramFile, header=True,index=False)

if ('lastStart.int' in os.listdir(os.path.realpath('.')) and
                        iteration == options.numiter+1):
    call(['rm','lastStart.int'])