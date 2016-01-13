# -*- coding: utf-8 -*-
"""
last modified 1/6/16
"""
import subprocess, os, math
import pandas as pd
from random import uniform
from optparse import OptionParser

from constants import * # sys.path is modified here
#import toolsPath
import collisionCheck, Sensors, Controllers

import traci
import traci.constants as tc

''' put the name of the SUMO config file to use here '''
CONFIGNAME = "inter1l"
outputName = 'intersim_b'
outputFolder = os.path.realpath('Results')
paramFolder = os.path.realpath('Parameters')

''' set sensor class here         '''
sensorToUse = Sensors.IdealSensor
randomizerToUse = None

dataFromTraciState = [tc.VAR_POSITION, tc.VAR_ANGLE, tc.VAR_SPEED]
dataFromTraciSet = [tc.VAR_LENGTH, tc.VAR_WIDTH, tc.VAR_ACCEL, tc.VAR_DECEL,
                    tc.VAR_MAXSPEED]

class Setting:
    verbose = False
setting = Setting()

class State:
    collisionOccurred = -1
    commEstablished = False
    step = 0
state = State()

class Output:
    def __init__(self, colnames):
        self.df = None
        self.colnames = colnames
    def add(self, newrows):
        if self.df is None:
            self.df = pd.DataFrame(newrows)
        else:
            self.df = self.df.append(newrows)
    def write(self, fileName, restart=False):
        self.df.columns =  self.colnames
        self.df.to_csv(fileName, sep=',', header=True, index=False)
        if restart:
            self.df = None
output = Output(['time','vehID','x','y','angle','speed'])
params = Output(['Ego Initial Speed','Alter Initial Speed','Collision Time',
                 'Colliding Vehicle'])

vehicleSetParams = {}
controllers = {}


def init(iteration = 0, defaultCONFIGNAME = CONFIGNAME):
    ## set up SUMO parameters
    optParser = OptionParser()
    optParser.add_option("-v", "--verbose", action="store_true", dest="verbose",
                         default=False, help="tell me what you are doing")
    optParser.add_option("-g", "--gui", action="store_true", dest="gui",
                         default=True, help="run with GUI")
    optParser.add_option("-c", "--config", type="string", dest="CONFIGNAME",
                         default=None, help="SUMO config to use")
    (options, args) = optParser.parse_args()
    setting.verbose = options.verbose
    
    completeCommand = [SUMO]
    if options.gui:
        completeCommand = [SUMOGUI]
    sumoConfig = defaultCONFIGNAME + "/" + defaultCONFIGNAME + ".sumocfg"
    if options.CONFIGNAME is not None:
        sumoConfig = options.CONFIGNAME+"/"+options.CONFIGNAME+".sumocfg"
    completeCommand += ["-c", sumoConfig] 
    if randomizerToUse is None:  # more subtle randomness still in progress
        completeCommand += ["--random"]
    
    ## start SUMO
    sumoProcess = subprocess.Popen(completeCommand, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT)
    traci.init(PORT, 4)
    traci.simulation.subscribe()

    ## loop
    try:
        while takeNextStep():
            doStep()
    finally:
        #if state.collisionOccurred < 0:        
        #    print "made it to the end without a collision"
        traci.close()
        sumoProcess.wait()
        if iteration > 0:
            respondWithCollision = state.collisionOccurred
            state.collisionOccurred = -1
            state.step = 0
            output.write(outputFolder+'/'+outputName+str(iteration)+'.csv',
                         restart=True)
            return  respondWithCollision


def takeNextStep():
    result = state.step < MAXSTEPS
    result &= traci.simulation.getMinExpectedNumber() > 0    
    result &= state.collisionOccurred - state.step <= 3 # 3 seconds after collision
    return result


def getFromSubscription(vehID, subs, varNames):
    ## extracts specified elements from subscription and returns
    ## them as a list.  Pre-processes the element when necessary.
    data = []
    data.append(vehID)
    for varName in varNames:
        if varName == tc.VAR_POSITION:
            data.append((subs[varName][0]))
            data.append(subs[varName][1])
        elif varName == tc.VAR_ANGLE:
            data.append(math.radians(subs[varName]))
        else:
            data.append(subs[varName])
    return data


def getSetParams(vehID):
    # these are vehicle variables that will not change over time
    params = []
    for varID in dataFromTraciSet:
        varValue = traci.vehicle._getUniversal(varID, vehID) * 1.0
        if randomizerToUse is not None:    
            # this is currently the best way to alter variables
            newValue = randomizerToUse(varID, varValue)
            if newValue != varValue:
                varValue = newValue
                resetParam(varID, varValue, vehID)
        params += [varValue]
    return params


def doStep():
    state.step += 1
    if setting.verbose:
        print "step", state.step
    #if state.step == 1:
    #    traci.gui.trackVehicle("View #0", "ego")
    traci.simulationStep()
    # adding new vehicles, if any    
    departed = traci.simulation.getSubscriptionResults()[tc.VAR_DEPARTED_VEHICLES_IDS]    
    for v in departed:
        traci.vehicle.subscribe(v,dataFromTraciState)
        vehicleSetParams[v] = getSetParams(v)
        if v=='ego':
            thisv = egov
        else:
            thisv = alterv
        controllers[v] = Controllers.Inter1Control(v, thisv)
        #getController(v, vehicleSetParams[v])
        if controllers[v] is not None:
            if setting.verbose:
                print "allowing forward and lane crashes for",v
            traci._sendIntCmd(tc.CMD_SET_VEHICLE_VARIABLE, tc.VAR_SPEEDSETMODE,
                              v, controllers[v].speedMode)
            traci.vehicle.setLaneChangeMode(v, controllers[v].laneChangeMode)
    # gather vehicle states for this step
    vStates = {}
    for vehID, subs in traci.vehicle.getSubscriptionResults().iteritems():        
        tempParams = getFromSubscription(vehID, subs, dataFromTraciState)
        vStates[vehID] = VState( tempParams + vehicleSetParams[vehID] )
    #
    for vehID, vState in vStates.iteritems():
        sensor = sensorToUse(vState)
        #
        for otherID, otherState in vStates.iteritems():
            if vehID != otherID:
                #
                # check for collisions
                if collisionCheck.check(vState, otherState):
                    #print "collision! pos",vState.x,vState.y,"step",state.step
                    if state.collisionOccurred < 0:
                        addCrashSymbol(vState, otherState)
                        state.collisionOccurred = state.step*.1
                    break
                #
                # update sensor
                sensor.addObstacle(otherState)
        #
        if not state.commEstablished and len(sensor.getObstacles()) > 0:
            addSensorSymbol()
            state.commEstablished = True
        if state.commEstablished and len(sensor.getObstacles()) == 0:
            state.commEstablished = False
            removeSensorSymbol()
        if setting.verbose:
            for vstat in sensor.getObstacles():
                print vehID, "detects", vstat.x, vstat.y,"speed",vstat.speed
        # store this vehicle movement
        output.add([[state.step*.1, vehID, vState.x, vState.y, vState.angle,
                     vState.speed]])
        # use data to decide speed
        if controllers[vehID] is not None:
            controllers[vehID].updateSpeed(vState.speed)
            commandedSpeed = controllers[vehID].nextStep(sensor.getObstacles())
            traci.vehicle.setSpeed(vehID, commandedSpeed)
            if setting.verbose:
                print "setting speed", commandedSpeed
                


def resetParam(varID, varValue, vehID):
    if varID == tc.VAR_LENGTH:
        traci.vehicle.setLength(vehID, varValue)
    elif varID == tc.VAR_WIDTH:
        traci.vehicle.setWidth(vehID, varValue)
    elif varID == tc.VAR_ACCEL:
        traci.vehicle.setAccel(vehID, varValue)
    elif varID == tc.VAR_DECEL:
        traci.vehicle.setDecel(vehID, varValue)
    elif varID == tc.VAR_MAXSPEED:
        traci.vehicle.setMaxSpeed(vehID, varValue)
    
def addCrashSymbol(aState,bState):
    acenterx = aState.x - 5./2.*math.sin(aState.angle)
    bcenterx = bState.x - 5./2.*math.sin(bState.angle)
    acentery = aState.y + 5./2.*math.cos(aState.angle)
    bcentery = bState.y + 5./2.*math.cos(bState.angle)
    center = (acenterx/2. + bcenterx/2., acentery/2. + bcentery/2.)
    circlePoints = []
    for k in range(16):
        angle = k*2*math.pi/16.
        radius = 3. + math.fmod(k,2)*2.
        circlePoints += [(center[0] + radius*math.cos(angle),
                          center[1] + radius*math.sin(angle))]
    traci.polygon.setShape('crashcircle', circlePoints)
    
def addSensorSymbol():
    traci.polygon.setColor('sensor',(0,255,0,0))
def removeSensorSymbol():
    traci.polygon.setColor('sensor',(255,255,255,0)) 

def rndSpeed():
    return uniform(25,45)*.447

if __name__ == "__main__":
    numiter=0
    
    if numiter == 0:
        egov = rndSpeed()
        alterv = rndSpeed()
        init(0)
    else:
        for it in range(numiter):
            egov = rndSpeed()
            alterv = rndSpeed()
            collide = init(it+1)
            params.add([[egov,alterv,collide, 'alter']])
        params.write(paramFolder+"/"+outputName+"_param.csv")