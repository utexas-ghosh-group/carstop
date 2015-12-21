# -*- coding: utf-8 -*-
from sumoMethodsWindows import Sumo
import pandas as pd
import numpy as np
#import os
import time
#from random import uniform
import collisionCheck
#from subprocess import call

numiter = 0

VEHsize = (5,2) # meters, length by width
DELTAT = .1
configuration = 'emptyInter2l'
MS2KPH = 3.6
roadOrder = [1,3,2,4,1,3,2] # counterclockwise

def discrete(x,y):
    return np.floor(x/float(y))*y

intersectionLookUp = [['1i_0','3o_0',':0_12_0'] , ['1i_0','2o_0',':0_13_0'] ,
                      ['1i_0','2o_1',':0_13_1'] , ['1i_1','4o_1',':0_14_0'] ,
                      ['2i_0','4o_0',':0_4_0'] , ['2i_0','1o_0',':0_5_0'] ,
                      ['2i_0','1o_1',':0_5_1'] , ['2i_1','3o_1',':0_7_0'] ,
                      ['3i_0','2o_0',':0_8_0'] , ['3i_0','4o_0',':0_9_0'] ,
                      ['3i_0','4o_1',':0_9_1'] , ['3i_1','1o_1',':0_11_0'] ,
                      ['4i_0','1o_0',':0_0_0'] , ['4i_0','3o_0',':0_1_0'] ,
                      ['4i_0','3o_1',':0_1_1'] , ['4i_1','2o_1',':0_3_0'] ]

intersectionInfo = pd.read_csv('inter2collisions.csv',header=0)

def splitRoad(lane):
    if lane[0]==':': # internal lane for intersection
        inroad = next((i for i,j,k in intersectionLookUp if k==lane))
        return [int(inroad[0]) , ':' , int(inroad[len(inroad)-1])]
    rt,ln = lane.split('_')
    return [int(rt[0]), rt[1], int(ln)]
    
def makeRoad(roadNum, io, ln):
    return str(roadNum)+io+'_'+str(ln)

class WriteFrame: # makes repeatedly adding to data frame easier
    def __init__(self, colnames=None):
        self.colnames = colnames
        self.restart=True
    def add(self, newrows):
        if self.restart:
            self.df = pd.DataFrame(newrows)
            self.restart=False
        else:
            self.df = self.df.append(newrows)
    def out(self):
        if self.colnames is not None:
            self.df.columns =  self.colnames
        #self.df.to_csv(fileName, sep=',', header=True, index=False)
        self.restart=True
        return self.df
QLearnInfo = WriteFrame(['ID','Car Location','Lane Number','Destination Road',
                         'Destination Lane','Correct Lane','Speed','Collision',
                         'Adjacent Lane Free','Front of Line',
                          'Speed of Ahead Car','Distance to Ahead Car',
                          'Time to Nearest Vehicle'])


iteration = 1
err = 0

## run iteration
while iteration <= numiter and err == 0:    
    starttime = time.time()
    Sim = Sumo(configuration)
    
    ## now set up parameters and vehicles
    # [creation time, starting lane, destination lane, starting speed]
    cars = pd.DataFrame([[0., '1i_0','3o_0',20.]])
    cars = cars.append([[0., '2i_0','1o_0',20.]])
    cars.columns = ['time','lane','dest','speed']
    
    ncars = cars.shape[0]
    cars['status'] = [-1]*ncars #-1 not created, 0 created, 1 exited, 2 crashed
    cars['lanepos'] = [-1]*ncars # not given to Agent but needed for calculation
    cars['x'] = [-1]*ncars
    cars['y'] = [-1]*ncars
    cars['angle'] = [-1]*ncars
    cars['ilane'] = ['']*ncars
    ttime = 0
    maxTime = 100 # seconds
    
    ## run simulation
    while ttime < maxTime and err == 0 and np.any(cars['status']<=0):
        
        # create new vehicles
        for carID in np.where(cars['status']<0)[0]:
            car = cars.iloc[carID]
            if car['time'] <= ttime:
                err += Sim.createVehicle(str(carID), car['lane'], 0.)
                # need to create an Agent?
                cars['status'].iloc[carID] = 0
        
        # gather vehicle info
        for carID in np.where(cars['status']==0)[0]:        
            carLane,carLanePos,carPos,carAngle = Sim.getVehicleState(str(carID))
            if carLane[1] == 'o':
                cars['status'] = 1
            cars['lane'].iloc[carID] = carLane
            cars['lanepos'].iloc[carID] = carLanePos
            cars['x'].iloc[carID] = carPos[0]
            cars['y'].iloc[carID] = carPos[1]
            cars['angle'].iloc[carID] = carAngle
            if carLane[1] == 'i': # not in intersection yet
                cars['ilane'].iloc[carID] = next((k for i,j,k in
                                intersectionLookUp if i==carLane and 
                                j==cars['dest'].iloc[carID]))
            else:
                cars['ilane'].iloc[carID] = carLane
        
        # check for collisions
        activeCars = np.where(cars['status']==0)[0]
        for carNum in range(len(activeCars)):
            carID = activeCars[carNum]
            car = cars.iloc[carID]
            for altNum in range(carNum):
                altID = activeCars[altNum]
                alt = cars.iloc[altID]
                carObject = pd.Series([car['x'],car['y'],car['angle'],5.,2.],
                                      index=collisionCheck.collisionVars)
                altObject = pd.Series([alt['x'],alt['y'],alt['angle'],5.,2.],
                                      index=collisionCheck.collisionVars)
                if collisionCheck.check(carObject, altObject):
                    cars['status'].iloc[carID] = 2
                    cars['status'].iloc[altID] = 2  
        
        
        # gather when the vehicles reach potential collision points
        trajectories = {}
        for carID in np.where(cars['status']==0)[0]:
            car = cars.iloc[carID]
            route,grade,ln = splitRoad(car['lane'])
            
            if grade=='i': # not in intersection yet
                initDist = car['lanepos'] - Sim.getLaneInfo(car['lane'])[0]
            else:
                initDist = car['lanepos']
            
            trajectory = []
            for crossIndex in range(intersectionInfo.shape[0]):
                thisCross = intersectionInfo.iloc[crossIndex]
                if thisCross['lane'] == car['ilane']:
                    timeToCrossingStart = (thisCross['begin_lp'] - 
                                            initDist) / car['speed']
                    timeToCrossingEnd = (thisCross['end_lp'] - 
                                            initDist) / car['speed']
                    trajectory += [[thisCross['lane2'],
                                    timeToCrossingStart, timeToCrossingEnd]]
            trajectories[carID] = trajectory
         
         
        # update everything
        for carID in np.where(cars['status']==0)[0]:
            car = cars.iloc[carID]
            route,grade,ln = splitRoad(car['lane'])
            destroute,o,destln = splitRoad(car['dest'])
            
            carUpdate = pd.Series()
            carUpdate['ID']=carID
            
            # whether the vehicle is entering the intersection (or already entered)
            if grade=='o':
                carUpdate['Car Location'] = 1
            elif grade=='i':
                carUpdate['Car Location'] = -1
            else:
                carUpdate['Car Location'] = 0
            # road and lane number for the vehicle (0 is right, 1 left)
            carUpdate['Road'] = route
            carUpdate['Lane Number'] = ln
            # destination road and lane
            carUpdate['Destination Road'] = destroute
            carUpdate['Destination Lane'] = destln
            # turn direction for the vehicle (-1 left, 0 straight, 1 right)
            curr = next((i for i,j in enumerate(roadOrder) if j==route))
            relativeOrder = next((i - curr for i,j in enumerate(roadOrder) 
                                                if j==destroute and i > curr))
            carUpdate['Turn Direction'] = 2 - relativeOrder
            # whether lane is correct for destination
            if ln == 0:
                carUpdate['Correct Lane'] = carUpdate['Turn Direction']>=0
            else:
                carUpdate['Correct Lane'] = carUpdate['Turn Direction']==-1
            # speed (discretized)
            carUpdate['Speed'] = discrete( car['speed']*MS2KPH, 10)
            # whether car collided
            carUpdate['Collision'] = False
            # adjacent lane is free (no collision if there is a lane change)
            if grade=='i':
                timeToLookAhead = 1.
                carUpdate['Adjacent Lane Free'] = 1 + ln
                adjlane = makeRoad(route,'i',1 - ln)
                searchAdjacent = (cars['status']==0)&(cars['lane']==adjlane)
                for otherCarID in np.where(searchAdjacent)[0]:
                    alt = cars.iloc[otherCarID]
                    carpositions = (np.arange(0,timeToLookAhead,DELTAT) *
                                    car['speed'] + car['lanepos'])   
                    altpositions = (np.arange(0,timeToLookAhead,DELTAT) *
                                    alt['speed'] + alt['lanepos'])                
                    if np.any(np.abs(altpositions - carpositions) <= 5.):
                        carUpdate['Adjacent Lane Free'] = 0
            # closeness of nearest vehicle - not complete        
            ttC = 100
            for crossing in trajectories[carID]: 
                altRoad, beginTime, endTime = crossing
                collideVehicles = (cars['status']==0)
                collideVehicles = collideVehicles*(cars['ilane']==altRoad) > 0
                for altcar in np.where(collideVehicles)[0]:
                    for altcrossing in trajectories[altcar].shape[0]:
                        origRoad,altbeginTime,altendTime = trajectories[altcar][altcrossing]
                        if origRoad == car['ilane']:
                            if endTime > altbeginTime or altendTime > beginTime:
                                ttC = min(ttC, beginTime)
            carUpdate['Time to Nearest Vehicle'] = ttC
            #carUpdate['Time to Nearest Vehicle'] = -1
            # info on ahead vehicle
            carUpdate['Next To Intersection'] = False
            aheadVehicles = (cars['status']==0)*(cars['lane']==car['lane'])>0
            aheadVehicles = aheadVehicles*(cars['lanepos']>car['lanepos'])>0
            aheadVehicles = np.where(aheadVehicles)[0]
            if len(aheadVehicles)==0:
                carUpdate['Speed of Ahead Car'] = -1
                carUpdate['Distance to Ahead Car'] = -1
                if carUpdate['Car Location']<0:
                    carUpdate['Next To Intersection'] = True
            else:
                closestVehicle = np.argmin(cars['lanepos'].iloc[aheadVehicles])
                closestCar = cars.iloc[aheadVehicles[closestVehicle]]
                carUpdate['Speed of Ahead Car'] = discrete(
                                        closestCar['speed']*MS2KPH, 10)
                carUpdate['Distance to Ahead Car'] = discrete(
                    (closestCar['lanepos']-car['lanepos'])/car['speed'], .5)
                if not closestCar['lane'][1] == 'i' and carUpdate['Car Location']<0:
                    carUpdate['Next To Intersection'] = True
                    
            # fffff
            # GiveToAgent(carID,carUpdate)
            QLearnInfo.add(pd.Series(carUpdate))
        
        # send current state space info to Q-learners
        #global updates <- QLearnInfo.out()
                
                
        # get actions
        carsThatAct = (cars['status'] == 0) + (cars['status'] == 1) > 0
        for carID in np.where(carsThatAct)[0]:
            car = cars.iloc[carID]
            route,grade,ln = splitRoad(car['lane'])
            #speedChange, laneChange, turn = ... Agent.getActions()
            # speedChange in [-10,0,10]
            # laneChange = True/False
            # turn = True/False, only activates when approaching intersection
            speedChange = 0
            turn = False
            laneChange = False
            
            car['speed'] = car['speed']+speedChange
            distance = car['speed']*DELTAT
            if turn and grade=='i':
                zz = list((j for i,j,k in intersectionLookUp if i==car['lane']))
                if len(zz) > 1:
                    if zz[0]==car['dest']:
                        newdest=zz[1]
                    else:
                        newdest=zz[0]
                #right = next((i for i,j in enumerate(roadOrder) if 
                #                        j==route)) + 1
                #newdest = makeRoad(roadOrder[right],'o',ln)
                #if not newdest == car['dest']:
                    err += Sim.moveVehicleAlong(str(carID), 0.0, newdest)
                    #car['dest'] = newdest # keep old destination?
            if laneChange and (grade =='i' or grade=='o'):
                newlane = makeRoad(route,grade,1-ln)
                err += Sim.moveVehicle(str(carID), newlane, car['lanepos'])
                cars['lane'] = newlane
                
            if Sim.getLaneInfo(car['lane'])[0] - carLanePos <= distance:
                if grade=='i': # entering intersection
                    #internal = next((k for i,j,k in intersectionLookUp if
                    #            i==car['lane'] and j==car['dest']))
                    #err += Sim.moveVehicleAlong(str(carID), distance, internal)
                    err += Sim.moveVehicleAlong(str(carID), distance, ':')
                elif grade=='o': # leaving simulation
                    Sim.removeVehicle(str(carID))
                    cars['status'].iloc[carID] = 1
                    # somehow inform Agents that car is gone
                else: # going from intersection to exit lane
                    err += Sim.moveVehicleAlong(str(carID), distance, car['dest'])
            elif distance > 0:
                err += Sim.moveVehicleAlong(str(carID), distance, car['lane'])
                
    
    if err == 0:
        # finish step
        Sim.end()        
        print "iteration "+str(iteration)+" : "+str(time.time()-starttime)
        iteration += 1
        time.sleep(.05)
    else:
        print "iteration "+str(iteration)+" failed, restart"