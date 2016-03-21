"""
3/20/16
"""
import pandas as pd
#import numpy as np
import collisionCheck

import sys,os
sys.path.append(os.path.realpath('../../simulator'))
from OwnSim import OwnSimulator as Simulator
from OwnSim import getLength as getRoadLength

VEHsize = (5.3,2.2) # meters, length by width
configuration = 'Qsim'

intersectionLookUp = [['1i_0','3o_0'] , #['1i_0','2o_1'] ,
                      ['1i_0','2o_0'] , ['1i_1','4o_1'] ,
                      ['2i_0','4o_0'] , #['2i_0','1o_1'] ,
                      ['2i_0','1o_0'] , ['2i_1','3o_1'] ,
                      ['3i_0','2o_0'] , #['3i_0','4o_1'] ,
                      ['3i_0','4o_0'] , ['3i_1','1o_1'] ,
                      ['4i_0','1o_0'] , #['4i_0','3o_1'] ,
                      ['4i_0','3o_0'] , ['4i_1','2o_1'] ]
roadOrder = pd.DataFrame({'left':[4,3,1,2],'right':[3,4,2,1],
                          'straight':[2,1,4,3]},index=[1,2,3,4])
                          
def combineRoad(inroad, outroad):
    if any(list(inroad==i and outroad==j for i,j in intersectionLookUp)):
        return inroad + '_' + outroad
    return None
    
class WriteFrame: # makes repeatedly adding to data frame easier
    def __init__(self, colnames=None):
        self.colnames = colnames
        self.restart=True
    def add(self, newrows):
        if self.restart:
            self.df = pd.DataFrame(newrows)
            self.restart=False
        else:
            self.df = self.df.append(newrows, ignore_index = True)
    def out(self):
        if self.colnames is not None:
            self.df.columns =  self.colnames
        #self.df.to_csv(fileName, sep=',', header=True, index=False)
        self.restart=True
        return self.df
    
    
Sim = Simulator('Qsim', gui=True, delay = .01)

# first gather the possible positions/angles of each intersection lane
allcars = {}
for start, end in intersectionLookUp:
    road = combineRoad(start, end)
    print "at "+road
    Sim.createVehicle(road, start, 90.)

    turn = 'straight'
    if roadOrder.loc[int(start[0]),'left'] == int(end[0]):
        turn = 'left'
    elif roadOrder.loc[int(start[0]),'right'] == int(end[0]):
        turn = 'right'
    
    thisdf = WriteFrame(['x','y','angle','length','width','lp'])    
    
    dist = 0.1
    status = -1
    totaldist = 0
    while status < 1 and totaldist < 50:
        lane,lanepos,pos,angle = Sim.getVehicleState(road)
        status = 1*(lane[1]=='o' and lanepos > 5.) - 1*(lane[1]=='i' and len(lane)==4)
        if lane[1]=='o': # must be included since back of vehicle
                lanepos = lanepos + getRoadLength(road)
        if status==0:  # add vehicle info to the dataframe
            thisdf.add([[pos[0],pos[1],angle,VEHsize[0],VEHsize[1],lanepos]])
        Sim.moveVehicleAlong(road, dist, turn)
        totaldist += dist
        Sim.updateGUI()
        
    allcars[road] = thisdf.out()
    Sim.removeVehicle(road)
Sim.end(waitOnEnd = False)

for car in allcars.itervalues():
    car['angle'] = car['angle'] + 1.5708

# now use dataframes of position+angle to find collision points
collisions = WriteFrame(['lane','lane2','begin_lp','end_lp'])
for an in range(len(intersectionLookUp)):
    for bn in range(an):
        print "checking "+str(an)+", "+str(bn)
        starta,enda = intersectionLookUp[an]
        startb,endb = intersectionLookUp[bn]
        roada = combineRoad(starta,enda)
        roadb = combineRoad(startb,endb)
        cara = allcars[roada]
        carb = allcars[roadb]
        
        abegin = 50
        aend = -1
        bbegin = 50
        bend = -1
        for acount in range(cara.shape[0]):
            for bcount in range(carb.shape[0]):
                apos = cara['lp'].iloc[acount]
                bpos = carb['lp'].iloc[bcount]
                if collisionCheck.check(cara.iloc[acount], carb.iloc[bcount]):
                    if abegin > apos:
                        abegin = apos
                    if bbegin > bpos:
                        bbegin = bpos
                    if aend < apos:
                        aend = apos
                    if bend < bpos:
                        bend = bpos
        if aend > 0 and not starta==startb: # ignore cars from same lane
            collisions.add([[roada,roadb,abegin,aend],[roadb,roada,bbegin,bend]])
        
collisions.out().to_csv('collisionsFile_OwnSim.csv', sep=",",
                header=True, index=False)