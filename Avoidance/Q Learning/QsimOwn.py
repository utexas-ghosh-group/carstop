"""
4/5/16
"""
# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
import time
import collisionCheck
from collisionHull import minTTC
from Qsimformat import formatState
from QsimInit import initialize
### import QTableIndex
from Agent import Agents as Agent
from Agent import updateQvalue

import sys,os
sys.path.append(os.path.realpath('../../simulator'))
from OwnSim import Simulator
from OwnSim import RoadMap

numiter = 1
QTable = np.zeros((12096,18)) # run these two lines to initialize
#np.save('qtable.npy',QTable)      # a blank table and stats list
#QTable = np.load('qtable.npy')
#Stats_train.to_csv('stats_training.csv')
#Stats_train = pd.read_csv('stats_training.csv')
#Stats_train=pd.DataFrame(np.zeros([1,5]),columns=['Collision','Reward','Time','WrongPath','Cars'])
Stats_train=pd.Series([0.]*5,index=['Collision','Reward','Time','WrongPath','Cars'])


VEHsize = (5.,2.) # meters, length by width
DELTAT = .1
MS2KPH = 3.6
roadOrder = pd.DataFrame({'straight':[2,1,4,3],'right':[3,4,2,1],
                          'left':[4,3,1,2]},index=[1,2,3,4])

#intersectionLookUp = [['1i_0','3o_0'] , ['1i_0','2o_0'] ,
#                      ['1i_0','2o_1'] , ['1i_1','4o_1'] ,
#                      ['2i_0','4o_0'] , ['2i_0','1o_0'] ,
#                      ['2i_0','1o_1'] , ['2i_1','3o_1'] ,
#                      ['3i_0','2o_0'] , ['3i_0','4o_0'] ,
#                      ['3i_0','4o_1'] , ['3i_1','1o_1'] ,
#                      ['4i_0','1o_0'] , ['4i_0','3o_0'] ,
#                      ['4i_0','3o_1'] , ['4i_1','2o_1'] ]
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
intersections   =    [['1i_0','3o_0'] , #['1i_0','2o_1'] ,
                      ['1i_0','2o_0'] , ['1i_1','4o_1'] ,
                      ['2i_0','4o_0'] , #['2i_0','1o_1'] ,
                      ['2i_0','1o_0'] , ['2i_1','3o_1'] ,
                      ['3i_0','2o_0'] , #['3i_0','4o_1'] ,
                      ['3i_0','4o_0'] , ['3i_1','1o_1'] ,
                      ['4i_0','1o_0'] , #['4i_0','3o_1'] ,
                      ['4i_0','3o_0'] , ['4i_1','2o_1'] ]  
roadMap = RoadMap(roads, intersections)

#intersectionInfo = pd.read_csv('collisionsFile_OwnSim.csv',header=0)
intersectionInfo = pd.read_csv('collisionsFile_Quad.csv',header=0) 
   
def splitRoad(lane):
    if len(lane)==9: # internal lane for intersection
        #inroute,ingrade,inlane = splitRoad(lane[:4])
        return [lane[:4], 1, lane[5:]]
    elif len(lane)==4: # starting or ending road
        if lane[1] == 'i':
            grade = 0
        elif lane[1] == 'o':
            grade = 2
    return [int(lane[0]), grade, int(lane[3])]
    
def makeRoad(roadNum, grade, ln):
    if grade == 2:
        return str(roadNum)+'o_'+str(ln)
    elif grade == 0:
        return str(roadNum)+'i_'+str(ln)
    raise 'cant make road with grade 1'


def gatherStateInfo(cars, trajectories, carID):
    car = cars.iloc[carID]
    if len(car['lane']) == 9:
        route,grade,ln = splitRoad(car['lane'][:4])
        grade = 1
    else:
        route,grade,ln = splitRoad(car['lane'])
    destroute,o,destln = splitRoad(car['dest'])
    
    carUpdate = pd.Series()
    carUpdate['ID'] = carID
    # whether the vehicle is entering the intersection (or already entered)
    if grade==2:
        carUpdate['Car Location'] = 2
    elif grade==0:
        carUpdate['Car Location'] = 0
    else:
        carUpdate['Car Location'] = 1
    # road and lane number for the vehicle (0 is right, 1 left)
    carUpdate['Road'] = route
    carUpdate['Lane Number'] = ln
    # destination road and lane
    carUpdate['Destination Road'] = destroute
    carUpdate['Destination Lane'] = destln
    # turn direction for the vehicle (0 left, 1 straight, 2 right)
    if destroute == roadOrder.loc[route].left:
        carUpdate['Turn Direction'] = 0
    elif destroute == roadOrder.loc[route].straight:
        carUpdate['Turn Direction'] = 1
    else:
        carUpdate['Turn Direction'] = 2
    # whether lane is correct for destination
    if ln == 0:
        carUpdate['Correct Lane'] = carUpdate['Turn Direction']>=1
    else:
        carUpdate['Correct Lane'] = carUpdate['Turn Direction']==0
    # speed (discretized)
    carUpdate['Speed'] = car['speed']*MS2KPH
    # whether car collided
    carUpdate['Collision'] = car['status'] >= 2
    carUpdate['Colliding Vehicle'] = car['status'] - 2
    # adjacent lane is free (no collision if there is a lane change)
    if grade==0 or grade==2:
        timeToLookAhead = 1.
        carUpdate['Adjacent Lane Free'] = 1 + ln
        adjlane = makeRoad(route,grade,1 - ln)
        searchAdjacent = (cars['status']==0)&(cars['lane']==adjlane)
        for otherCarID in np.where(searchAdjacent)[0]:
            alt = cars.iloc[otherCarID]
            carpositions = (np.arange(0,timeToLookAhead,DELTAT) *
                            car['speed'] + car['lanepos'])   
            altpositions = (np.arange(0,timeToLookAhead,DELTAT) *
                            alt['speed'] + alt['lanepos'])                
            if np.any(np.abs(altpositions - carpositions) <= VEHsize[0]):
                carUpdate['Adjacent Lane Free'] = 0
    else:
        carUpdate['Adjacent Lane Free'] = 0
        
    carUpdate['Time to Nearest Vehicle'] = -1
    # find closest crossing vehicles        
    tNV = 100
    closestCar = -1
    secondTNV = 100
    for crossing in trajectories[carID]:
        altRoad = crossing[0]
        #beginTime, endTime = crossing[1:]
        egospeed,egodist = crossing[1:3]
        egohull = crossing[3:]        
        collideVehicles = (cars['status']==0)
        collideVehicles = collideVehicles & ((cars['ilane']==altRoad) > 0)
        for altcar in np.where(collideVehicles)[0]:
            for altcrossing in trajectories[altcar]:
                origRoad = altcrossing[0]
                if origRoad == car['ilane']:
#                    altbeginTime,altendTime = altcrossing[1:]
#                    ttc = 100
#                    if endTime > altbeginTime and altendTime > beginTime:
#                        ttc = beginTime
                    altspeed,altdist = altcrossing[1:3]
                    althull = altcrossing[:2:-1]
                    combinedHull = list(((egohull[i],althull[i]) for i in
                                            range(len(egohull))))
                    ttc = minTTC(combinedHull,[egospeed,altspeed],
                                               [egodist,altdist], margin=.3)
                    if ttc < tNV:
                        secondTNV = tNV
                        tNV = ttc
                        closestCar = altcar
                    elif ttc < secondTNV:
                        secondTNV = ttc                          
    carUpdate['Time to Crossing Car'] = tNV
    carUpdate['Crossing Car'] = closestCar
    carUpdate['Time to 2nd Crossing Car'] = secondTNV
    
    # locate vehicles ahead of this one
    aheadLanes = []
    #futureAheadLane = None # without discounted reward this has no effect
    for joint in intersections:
        if joint[0] == car['lane']:
            aheadLanes += [roadMap.combineRoad(joint[0],joint[1])]
            #if joint[1] == car['dest']: # include ahead lane only if it is known dest.
            #    futureAheadLane = joint[1]
        if roadMap.combineRoad(joint[0],joint[1]) == car['lane']:
            aheadLanes = [joint[1]]
    aheadVehicles = (cars['status']==0) & (cars['lane']==car['lane'])
    aheadVehicles = aheadVehicles & (cars.index != carID) # ensure this vehicle is not included
    for aheadLane in aheadLanes:    
        aheadVehicles = aheadVehicles | (
                (cars['status']==0) & (cars['lane']==aheadLane) )
    aheadVehicles = np.where(aheadVehicles)[0]
    # locate closest and 2nd closest vehicles
    closestPos = 1000
    closestCar = None
    secondClosestPos = 1000
    secondClosestCar = None
    for j in aheadVehicles:
        aheadCar = cars.iloc[j]
        if aheadCar['lane'] == car['lane']:
            position = aheadCar['lanepos'] - car['lanepos']
            if position < 0:
                position = 1001
        else: # in lane ahead
            position = aheadCar['lanepos'] - car['lanepos'] +\
                           roadMap.getLength(car['lane'])
        # for lane further ahead:
        #position = aheadCar['lanepos'] - car['lanepos'] +\
        #           getRoadLength(car['lane']) + getRoadLength(aheadLanes[0])
        if position < closestPos:
            secondClosestPos = closestPos
            secondClosestCar = closestCar
            closestPos = position
            closestCar = j
        elif position < secondClosestPos:
            secondClosestPos = position
            secondClosestCar = j
    closestPos = closestPos - VEHsize[0]
    secondClosestPos = secondClosestPos - VEHsize[0]
    # gather info on ahead vehicles
    carUpdate['Next to Intersection'] = False
    carUpdate['2nd Next to Intersection'] = False    
    carUpdate['Ahead Car'] = -1
    carUpdate['Speed of Ahead Car'] = -1
    carUpdate['Distance to Ahead Car'] = -1
    carUpdate['Time to Ahead Car'] = 100
    carUpdate['Speed of 2nd Ahead Car'] = -1
    carUpdate['Distance to 2nd Ahead Car'] = -1
    carUpdate['Time to 2nd Ahead Car'] = 100
    if closestCar is None:
        if carUpdate['Car Location'] == 0:
            carUpdate['Next to Intersection'] = True        
    else:
        closestCarSpeed = cars.iloc[closestCar].speed
        carUpdate['Ahead Car'] = closestCar        
        carUpdate['Speed of Ahead Car'] = closestCarSpeed*MS2KPH
        carUpdate['Distance to Ahead Car'] = closestPos/car['speed']
        tNV = closestPos / (car['speed'] - closestCarSpeed)
        if tNV < 0: # never colliding
            tNV = 100
        carUpdate['Time to Ahead Car'] = tNV
        if carUpdate['Car Location'] == 0:
            if splitRoad(cars.iloc[closestCar].lane)[1] != 0:
                cars['Next to Intersection'] = True
        if secondClosestCar is None:
            carUpdate['Speed of 2nd Ahead Car'] = -1
            carUpdate['Distance to 2nd Ahead Car'] = -1
            if (carUpdate['Car Location'] == 0 and 
                carUpdate['Next to Intersection'] == 0):
                carUpdate['2nd Next to Intersection'] = True
        else:
            secondCCSpeed = cars.iloc[secondClosestCar].speed
            carUpdate['Speed of 2nd Ahead Car'] = secondCCSpeed * MS2KPH
            carUpdate['Distance to 2nd Ahead Car'] = secondClosestPos/secondCCSpeed
            secondTNV = secondClosestPos/(car['speed'] - secondCCSpeed)
            if secondTNV < 0: # never colliding
                secondTNV = 100
            carUpdate['Time to 2nd Ahead Car'] = secondTNV

    # locate behind vehicles
    behindLanes = []
    #pastBehindLane = None # without discounted reward this has no effect
    for joint in intersections:
        if joint[1] == car['lane']:
            behindLanes += [roadMap.combineRoad(joint[0],joint[1])]
        if roadMap.combineRoad(joint[0],joint[1]) == car['lane']:
            behindLanes = [joint[0]]
    behindVehicles = (cars['status']==0) & (cars['lane']==car['lane'])
    behindVehicles = behindVehicles & (cars.index != carID) # ensure this vehicle is not included
    if len(behindLanes) >= 1:
        behindVehicles = behindVehicles | (
                (cars['status']==0) & (cars['lane']==behindLanes[0]) )
    if len(behindLanes) == 2:
        behindVehicles = behindVehicles | (
                (cars['status']==0) & (cars['lane']==behindLanes[1]) )
    behindVehicles = np.where(behindVehicles)[0]
    # locate closest and 2nd closest vehicles
    closestPos = 1000
    closestCar = None
    secondClosestPos = 1000
    secondClosestCar = None
    for j in behindVehicles:
        behindCar = cars.iloc[j]
        if behindCar['lane'] == car['lane']:
            position = car['lanepos'] - behindCar['lanepos']
            if position < 0:
                position = 1001
        elif len(behindLanes) >= 1 and behindCar['lane'] == behindLanes[0]:
            position = car['lanepos'] - behindCar['lanepos'] +\
                       roadMap.getLength(behindLanes[0])
        elif len(behindLanes) == 2 and behindCar['lane'] == behindLanes[1]:
            position = car['lanepos'] - behindCar['lanepos'] +\
                       roadMap.getLength(behindLanes[1])
            #position = car['lanepos'] - behindCar['lanepos'] +\
            #           getRoadLength(behindLanes[0]) + getRoadLength(behindLanes[1])
        if position < closestPos:
            secondClosestPos = closestPos
            secondClosestCar = closestCar
            closestPos = position
            closestCar = j
        elif position < secondClosestPos:
            secondClosestPos = position
            secondClosestCar = j
    carUpdate['Behind Car'] = -1
    carUpdate['Time to Behind Car'] = 100
    carUpdate['Time to 2nd Behind Car'] = 100
    if closestCar is not None:
        carUpdate['Behind Car'] = closestCar        
        tNV = (closestPos - VEHsize[0]) / (
                            cars.iloc[closestCar].speed - car['speed'])
        if tNV < 0: # never colliding
            tNV = 100
        carUpdate['Time to Behind Car'] = tNV
    if secondClosestCar is not None:      
        secondTNV = (secondClosestPos - VEHsize[0]) / (
                cars.iloc[secondClosestCar].speed - car['speed'])
        if secondTNV < 0: # never colliding
            secondTNV = 100
        carUpdate['Time to 2nd Behind Car'] = secondTNV
        
    carUpdate['On Grid'] = True
    return carUpdate


## main code

iteration = 1

## run iteration
while iteration <= numiter:    
    starttime = time.time()
    print 'iteration '+ str(iteration)
    # set up parameters and vehicles
    cars = initialize(iteration)
    #cars1 = cars
    agents = {}
    for carID in range(cars.shape[0]):
        car = cars.iloc[carID]
        agents[carID] = Agent(carID, car['lane'],car['dest'],car['speed'],car['time'])
    
    ncars = cars.shape[0]
    cars['status'] = [-1]*ncars #-1 not created, 0 created, 1 exited, 2 crashed
    cars['lanepos'] = [-1]*ncars # not given to Agent but needed for calculation
    cars['x'] = [-1]*ncars
    cars['y'] = [-1]*ncars
    cars['angle'] = [-1]*ncars
    cars['ilane'] = ['']*ncars
    cars['lanechanged'] = [0]*ncars
    cars['changex'] = [0]*ncars
    cars['changey'] = [0]*ncars
    cars['changeangle'] = [0]*ncars

    collisionCount = 0
    TotReward = 0
    WrongPath = 0
    ttime = 0
    maxTime = 100 # seconds
       
    # start simulator
    Sim = Simulator(roadMap, gui = True, delay = .2)    

    ## run simulation
    while ttime < maxTime and np.any(cars['status']<=0):
        
        # create new vehicles
        for carID in np.where(cars['status']<0)[0]:
            car = cars.iloc[carID]
            if car['time'] <= ttime:
                # search through lane and see if another vehicle is taking the spot
                otherCars = (cars['status'] == 0) | (cars['status'] >= 2)
                addThisCar = True
                for otherID in np.where(otherCars)[0]:
                    if cars['lane'].iloc[otherID] == car['lane']:
                        if cars['lanepos'].iloc[otherID] <= 10.:
                            addThisCar = False
                if addThisCar:
                    Sim.createVehicle(str(carID), car['lane'], 0.)
                    cars.loc[carID,'status'] = 0
        
        # gather vehicle info
        for carID in np.where(cars['status']==0)[0]:        
            carLane,carLanePos,carPos,carAngle = Sim.getVehicleState(str(carID))
            
            if carLane == '':
                print '!!!! lane is empty'+ str(carID) + cars.loc[carID,'lane']
                carLane = cars.loc[carID,'lane']
                carLanePos = 0.0
                carPos = (-6.*carID,-6.*carID)
                carAngle = 0.
            elif carLane is None:
                print '!!!! lane is none '+ str(carID)
                cars.loc[carID,'status']=1
                continue
            
            #if carLane[1] == 'o':
            #    cars.loc[carID,'status'] = 1
            cars.loc[carID,'lane'] = carLane
            cars.loc[carID,'lanepos'] = carLanePos
            cars.loc[carID,'x'] = carPos[0]
            cars.loc[carID,'y'] = carPos[1]
            cars.loc[carID,'angle'] = carAngle
            
            if carLane[1] == 'i' and len(carLane)==4: # not in intersection yet
                destlane = cars.loc[carID,'dest']
                destlane = destlane[:3] + carLane[3:]
                cars.loc[carID,'ilane'] = roadMap.combineRoad(carLane, destlane)
            elif len(carLane)==4:
                cars.loc[carID,'ilane'] = carLane
            else:
                cars.loc[carID, 'ilane'] = carLane
        
        # cars that collided last time are now completely removed
        carsThatCollidedLastStep = np.where(cars['status']>=2)[0]
        for carID in carsThatCollidedLastStep:
            
            cars.loc[carID, 'status'] = 1
            Sim.removeVehicle(str(carID))
        
        # check for collisions
        activeCars = np.where(cars['status']==0)[0]
        for carNum in range(len(activeCars)):
            carID = activeCars[carNum]
            car = cars.iloc[carID]
            for altNum in range(carNum):
                altID = activeCars[altNum]
                alt = cars.iloc[altID]
                carObject = pd.Series([car['x'],car['y'],car['angle']+np.pi/2,
                                       VEHsize[0],VEHsize[1]],
                                       index=collisionCheck.collisionVars)
                altObject = pd.Series([alt['x'],alt['y'],alt['angle']+np.pi/2,
                                       VEHsize[0],VEHsize[1]],
                                       index=collisionCheck.collisionVars)
                if collisionCheck.check(carObject, altObject):
                    cars.loc[carID, 'status'] = 2 + altID
                    cars.loc[altID, 'status'] = 2 + carID
                    collisionCount += 2
                    Sim.addCrashSymbol(str(carID), str(altID))
                    #time.sleep(2.) # just to show that cars collided
        # check for double lane-change
        changedCars = np.where((cars['status']==0) & (cars['lanechanged']>0))[0]
        for carNum in range(len(changedCars)):
            carID = changedCars[carNum]
            car = cars.iloc[carID]
            for altNum in range(carNum):
                altID = activeCars[altNum]
                alt = cars.iloc[altID]
                carObject = pd.Series([car['changex'],car['changey'],car['changeangle'],
                                       VEHsize[0],VEHsize[1]],
                                       index=collisionCheck.collisionVars)
                altObject = pd.Series([alt['x'],alt['y'],alt['angle'],
                                       VEHsize[0],VEHsize[1]],
                                       index=collisionCheck.collisionVars)
                if collisionCheck.check(carObject, altObject):
                    cars.loc[carID, 'status'] = 2 + altID
                    cars.loc[altID, 'status'] = 2 + carID
                    collisionCount += 2
                    Sim.addCrashSymbol(str(carID), str(altID))
                    print "found a double lane-change collision"
                    print "shouldn't be happening anymore!!"
        
        # gather when the vehicles reach potential collision points
        trajectories = {}
        for carID in np.where((cars['status']==0) | (cars['status']>=2))[0]:
            car = cars.iloc[carID]
            route,grade,ln = splitRoad(car['lane'])
            
            if grade==0: # not in intersection yet
                initDist = car['lanepos'] - roadMap.getLength(car['lane'])
            elif grade==2: # left intersection (but maybe tail end is still there)
                initDist = car['lanepos'] + roadMap.getLength(car['ilane'])            
            else:
                initDist = car['lanepos']
            
            trajectory = []
            for crossIndex in range(intersectionInfo.shape[0]):
                thisCross = intersectionInfo.iloc[crossIndex]
                if thisCross['lane'] == car['ilane']:
#                   if thisCross['end_lp'] > initDist:
#                      timeToCrossingStart = (thisCross['begin_lp'] - 
#                                             initDist) / car['speed']
#                      timeToCrossingEnd = (thisCross['end_lp'] - 
#                                               initDist) / car['speed']
#                      trajectory += [[thisCross['lane2'],
#                                    timeToCrossingStart, timeToCrossingEnd]]
                    trajectory += [[thisCross['lane2'],car['speed'],initDist,
                                    thisCross['p1'],thisCross['p2'],
                                    thisCross['p3'],thisCross['p4']]]
            trajectories[carID] = trajectory
         
         
        carsToUpdate = np.where((cars['status']==0) | (cars['status']>=2))[0]        
         
        # update QTable
        Qstates = {}
        prevIndices = {}
        for carID in carsToUpdate:
            carUpdate = gatherStateInfo(cars, trajectories, carID)
            Qstates[carID] = carUpdate
            prevIndices[carID] = (agents[carID].state, agents[carID].action)
        # first gather normal reward
        #realReward = sum(reward(formatState(state)) for
        #                            state in Qstates.itervalues())
            
        realReward = 0
        for carID in carsToUpdate:
            agents[carID].state_space(*formatState(Qstates[carID]))
            realReward += agents[carID].reward(ttime)
        TotReward += realReward
        #print "all cars reward "+str(realReward)
        # rewards without vehicle
        for carID in carsToUpdate:
            differenceReward = 0
            for otherCar in carsToUpdate:
                if otherCar != carID:
                    agents[otherCar].state_space(*formatState(
                                    Qstates[otherCar], AgentToRemove=carID))
                    differenceReward += agents[otherCar].reward_def()
            if prevIndices[carID][0] >= 0:
                finalValue = updateQvalue(QTable[prevIndices[carID]],
                                          realReward - differenceReward)
                printString = "car reward "+str(carID) + ": reward "
                printString += str(realReward-differenceReward)
                printString += " new value " + str(finalValue)
                #print printString
                QTable[prevIndices[carID]] = finalValue
            #differenceReward = sum(reward(formatState(state, carID)) for
            #                        state in Qstates.itervalues())
            #if cars['indexS'][carID] >= 0:
            #    QTable[cars['indexS'][carID], cars['indexA'][carID]] += (
            #                realReward - differenceReward )
            #cars['indexS'][carID] = QTableIndex.stateToNum(formatState(Qstates[carID]))
        for carID in carsToUpdate: # have to redo this to reset all state variables
            agents[carID].state_space(*formatState(Qstates[carID]))
            printString = "car "+str(carID)
            printString += " NTI "+str(formatState(Qstates[carID]).iloc[7])+\
                            " turn "+str(formatState(Qstates[carID]).iloc[2])            
            printString += " cross "+str(Qstates[carID]['Time to Crossing Car'])
            printString += " ahead "+str(Qstates[carID]['Time to Ahead Car'])
            printString += " TTNV "+str(formatState(Qstates[carID]).iloc[7])
            printString += " Speed "+str(formatState(Qstates[carID]).iloc[4])
            
            #printString += " behind "+str(Qstates[carID]['Time to Behind Car'])
            print printString
                       
        ## make actions
        carsThatAct = cars['status'] == 0
        for carID in np.where(carsThatAct)[0]:
            car = cars.iloc[carID]
            route,grade,ln = splitRoad(car['lane'])
            # get action from agent
            laneChange, speedChange, turn = agents[carID].Action_training(
                                                    QTable)
            suppressLaneChange = False
            suppressLaneChange = suppressLaneChange or (grade == 2 and
                                    car['lanepos'] <= 5.)
            suppressLaneChange = suppressLaneChange or (grade == 0 and 
                                    car['lanepos'] >= 89.)
            if suppressLaneChange:
                laneChange = 0;
            printString = "car action "+str(carID)+": lane "+str(int(laneChange))
            printString += " speed "+str(int(speedChange))+" turn "+str(int(turn))
            #print printString
            #cars['indexA'][carID] = indexA
            
            offset = Sim.offset[str(carID)]
            maxChange = 1.
            if laneChange>0 or cars.loc[carID,'lanechanged']>0:
                cars.loc[carID,'lanechanged'] = 1
                newlane = None
                newoffset = None
                if ln == 1: # left lane
                    Sim.offsetVehicle(str(carID), maxChange)
                    if offset + maxChange > 3.35/2.: # crossed to right lane
                        crossed = 0
                        newlane = makeRoad(route, grade, 0)
                        newoffset = offset + maxChange - 3.35
                        cars.loc[carID,'lanechanged'] = 0
                else: # right lane
                    Sim.offsetVehicle(str(carID),-maxChange)
                    if offset - maxChange < -3.35/2.: # crossed to left lane
                        newlane = makeRoad(route, grade, 1)
                        newoffset = offset - maxChange + 3.35
                        cars.loc[carID,'lanechanged'] = 0
                if not newlane is None:
                    Sim.moveVehicle(str(carID), newlane, car['lanepos'], offset=newoffset)
                    cars.loc[carID,'lane'] = newlane
                cars.loc[carID,'changex'] = car['x']
                cars.loc[carID,'changey'] = car['y']
                cars.loc[carID,'changeangle'] = car['angle']
            else:
                #cars.loc[carID,'lanechanged'] = 0
                # head towards center of lane
                Sim.offsetVehicle(str(carID), min(max(-offset, -maxChange),maxChange))
                
            speedChange = [0,-10.,10.][int(speedChange)]/MS2KPH            
            cars.loc[carID,'speed'] = min(max(car['speed']+speedChange, 0.),60./MS2KPH)
            distance = car['speed']*DELTAT
            
            # the vehicle is about to make a transition to a new roa   
            exited = Sim.moveVehicleAlong(str(carID), distance, turn)
            if exited: # car leaving simulation
                Sim.removeVehicle(str(carID))
                cars.loc[carID,'status'] = 1
                WrongPath = WrongPath + int(route != splitRoad(car['dest'])[0])
        
        Sim.updateGUI(allowPause=True)
        ttime += 0.1

    # finish step
    Sim.end()        
    print "iteration "+str(iteration)+" : "+str(time.time()-starttime)
    print "collisions "+str(collisionCount)
    print "wrong "+str(WrongPath)
    Stats_train=Stats_train+[collisionCount,TotReward,ttime,WrongPath,float(cars.shape[0])]
    time.sleep(.2)
    iteration += 1

# save info afterwards
iteration = iteration - 1
#np.save('qtable_random2.npy',QTable)
Stats_train.loc['Collision'] = Stats_train.loc['Collision'] / Stats_train['Cars']
Stats_train.loc['Reward'] = Stats_train.loc['Reward'] / Stats_train['Cars']
Stats_train.loc['WrongPath'] = Stats_train.loc['WrongPath'] / Stats_train['Cars']
Stats_train.loc['Time'] = Stats_train.loc['Time']/iteration
statsSaveName = 'ownsim_train_'+str(iteration)+'.csv'
Stats_train.to_csv(statsSaveName)