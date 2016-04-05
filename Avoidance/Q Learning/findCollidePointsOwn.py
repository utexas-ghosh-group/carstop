"""
4/5/16
"""
import pandas as pd
#import numpy as np
import collisionCheck
import collisionHull

import sys,os
sys.path.append(os.path.realpath('../../simulator'))
from OwnSim import Simulator
from OwnSim import RoadMap

VEHsize = (5.3,2.2) # meters, length by width
configuration = 'Qsim'

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
roadOrder = pd.DataFrame({'left':[4,3,1,2],'right':[3,4,2,1],
                          'straight':[2,1,4,3]},index=[1,2,3,4])
                          
def combineRoad(inroad, outroad):
    if any(list(inroad==i and outroad==j for i,j in intersections)):
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
    
    
Sim = Simulator(roadMap, gui=False, delay = 0.)

# first gather the possible positions/angles of each intersection lane
allcars = {}
for start, end in intersections:
    road = combineRoad(start, end)
    print "at "+road
    Sim.createVehicle(road, start, 90.)

    turn = 0
    if roadOrder.loc[int(start[0]),'left'] == int(end[0]):
        turn = 2
    elif roadOrder.loc[int(start[0]),'right'] == int(end[0]):
        turn = 1
    
    thisdf = WriteFrame(['x','y','angle','length','width','lp'])    
    
    dist = 0.1
    status = -1
    totaldist = 0
    while status < 1 and totaldist < 50:
        lane,lanepos,pos,angle = Sim.getVehicleState(road)
        status = 1*(lane[1]=='o' and lanepos > 5.) - 1*(lane[1]=='i' and len(lane)==4)
        if lane[1]=='o': # must be included since back of vehicle
                lanepos = lanepos + roadMap.getLength(road)
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
#
## now use dataframes of position+angle to find collision points
#collisions = WriteFrame(['lane','lane2','begin_lp','end_lp'])
#for an in range(len(intersections)):
#    for bn in range(an):
#        print "checking "+str(an)+", "+str(bn)
#        starta,enda = intersections[an]
#        startb,endb = intersections[bn]
#        roada = combineRoad(starta,enda)
#        roadb = combineRoad(startb,endb)
#        cara = allcars[roada]
#        carb = allcars[roadb]
#        
#        abegin = 50
#        aend = -1
#        bbegin = 50
#        bend = -1
#        for acount in range(cara.shape[0]):
#            for bcount in range(carb.shape[0]):
#                apos = cara['lp'].iloc[acount]
#                bpos = carb['lp'].iloc[bcount]
#                if collisionCheck.check(cara.iloc[acount], carb.iloc[bcount]):
#                    if abegin > apos:
#                        abegin = apos
#                    if bbegin > bpos:
#                        bbegin = bpos
#                    if aend < apos:
#                        aend = apos
#                    if bend < bpos:
#                        bend = bpos
#        if aend > 0 and not starta==startb: # ignore cars from same lane
#            collisions.add([[roada,roadb,abegin,aend],[roadb,roada,bbegin,bend]])
#        
#collisions.out().to_csv('collisionsFile_OwnSim.csv', sep=",",
#                header=True, index=False)
                
               
collisions = WriteFrame(['lane','lane2','p1','p2','p3','p4'])
for an in range(len(intersections)):
    for bn in range(an):
        print "checking "+str(an)+", "+str(bn)
        starta,enda = intersections[an]
        startb,endb = intersections[bn]
        roada = combineRoad(starta,enda)
        roadb = combineRoad(startb,endb)
        cara = allcars[roada]
        carb = allcars[roadb]
        
        points = []
        for acount in range(cara.shape[0]):
            for bcount in range(carb.shape[0]):
                apos = cara['lp'].iloc[acount]
                bpos = carb['lp'].iloc[bcount]
                if collisionCheck.check(cara.iloc[acount], carb.iloc[bcount]):
                    points += [(apos, bpos)]
        if len(points) > 0:        
            points = collisionHull.getHull(points)
            collisions.add([[roada,roadb, points[0][0],points[1][0],
                             points[2][0],points[3][0]], [roadb,roada,
                             points[3][1],points[2][1],points[1][1],
                             points[0][1]]])
collisions.out().to_csv('collisionsFile_Quad.csv',sep=",",
                            header=True, index=False)