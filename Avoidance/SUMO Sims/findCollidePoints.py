# -*- coding: utf-8 -*-
from sumoMethods import Sumo
import pandas as pd
#import numpy as np
import collisionCheck
import time

VEHsize = (5.,2.) # meters, length by width
configuration = 'Qsim'
roadOrder = [1,3,2,4,1,3,2] # counterclockwise


intersectionLookUp = [['1i_0','3o_0',':0_12_0'] , ['1i_0','2o_0',':0_13_0'] ,
                      ['1i_0','2o_1',':0_13_1'] , ['1i_1','4o_1',':0_14_0'] ,
                      ['2i_0','4o_0',':0_4_0'] , ['2i_0','1o_0',':0_5_0'] ,
                      ['2i_0','1o_1',':0_5_1'] , ['2i_1','3o_1',':0_7_0'] ,
                      ['3i_0','2o_0',':0_8_0'] , ['3i_0','4o_0',':0_9_0'] ,
                      ['3i_0','4o_1',':0_9_1'] , ['3i_1','1o_1',':0_11_0'] ,
                      ['4i_0','1o_0',':0_0_0'] , ['4i_0','3o_0',':0_1_0'] ,
                      ['4i_0','3o_1',':0_1_1'] , ['4i_1','2o_1',':0_3_0'] ]

#intersectionLookUp = []
    
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
    
err = 0
Sim = Sumo(configuration,gui=True)

# first gather the possible positions/angles of each intersection lane
allcars = {}
for start, end, road in intersectionLookUp:
    print "at "+road
    err += Sim.createVehicle(road, start, 90.)
    err += Sim.moveVehicleAlong(road, 0.1, end)
    thisdf = WriteFrame(['x','y','angle','length','width','lp'])    
    
    dist = 1.
    status = -1
    totaldist = 0
    while status < 1 and totaldist < 50:
        lane,lanepos,pos,angle = Sim.getVehicleState(road)
        status = (lane[1]=='o')-(lane[1]=='i')
        if status == 0:  #add vehicle info to the dataframe
            thisdf.add([[pos[0],pos[1],angle,VEHsize[0],VEHsize[1],lanepos]])
        Sim.moveVehicleAlong(road, dist)
        totaldist += dist
        time.sleep(.05) # might guard against freeze
        
    allcars[road] = thisdf.out()
    Sim.removeVehicle(road)
Sim.end()

# now use dataframes of position+angle to find collision points
collisions = WriteFrame(['lane','lane2','begin_lp','end_lp'])
for an in range(len(intersectionLookUp)):
    for bn in range(an):
        print "checking "+str(an)+", "+str(bn)
        starta,enda,roada = intersectionLookUp[an]
        startb,endb,roadb = intersectionLookUp[bn]
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
        
collisions.out().to_csv('Qsim/inter2collisions.csv', sep=",",
                header=True, index=False)