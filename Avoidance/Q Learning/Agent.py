# -*- coding: utf-8 -*-
"""
Created on Fri Dec 11 13:19:17 2015

@author: Rahi
"""
from scipy.stats import bernoulli
from random import randrange, uniform
import random 
import math
import numpy as np

class Agents(object):
    """A customer of ABC Bank with a checking account. Customers have the
    following properties:

    Attributes:
        name: A string representing the customer's name.
        balance: A float tracking the current balance of the customer's account.
    """
    # total number of the cars on grid is drawn from a poisson distribution 
    # each car aslo assigned some initial speed which is drwan from a Gaussian distribution 
    # if 2 cars to start in the same lane, they should start with MinTimeInterval time differnce 
    def __init__(self, agent_id, source_lane, destination_lane, InitialSpeed,  MinTimeInterval):
        """Return a Customer object whose name is *name* and starting
        balance is *balance*."""
        self.name = agent_id
        self.source = source_lane
        self.destination = destination_lane
        self.initial_speed = InitialSpeed
        self.time_interval = MinTimeInterval

        self.Rtime = 0
        self.Rdest = 0
        self.RTTC = 0
        self.Rcoll = 0
        
        self.state = -1
        self.action = -1
        
        
        
    # Agent Location={Pre-intersection, At-intersection, Post-intersection}={0,1,2}
    # Agent Road = {left, right, down, up}={0,1,2,3}
    # Agent Lane=(in this case){right,left }= {0,1}
    # Destination Road = {left, right, down, up}={0,1,2,3}
    # Destination Lane=(in this case){right,left }= {0,1}    
    # Turn Direction = {left, stright,right} = {0,1,2}
    # Adjacent Lane is free ={none, left only, right only, both}={0,1,2,3}
    # Correct Lane = {no, yes} ={0,1}
    # Agent On  Grid = { yes,no} ={1,0}
    # speed = {0,10,20,30,40,50,60}={0,1,2,3,4,5,6}
    # collision ={no,yes}={0,1}
    # relative distance speed = {-3a,-2a,-a,0,a,2a,3a}={0,1,2,3,4,5,6,7}
    # DistancetoToCarOfInterest ={0,2,4,8,16,32,64,128,256 and more}
    # if 
    def state_space(self, AgentLocation, AgentLane, TurnDirection,
                    CorrectLane, Speed, Collision, Adj_LaneFree, TimeToNearestVehicle,
                    NextToIntersection, AgentonGrid):
        
        self.Location = AgentLocation
        #self.Road = AgentRoad
        self.Lane = AgentLane
        #self.DRoad = DestinationRoad
        #self.DLane = DestinationLane
        self.Turn = TurnDirection
        self.RLane = CorrectLane
        self.collision = Collision
        self.AdjFreeLane = Adj_LaneFree
        self.OnGrid = AgentonGrid
        self.speed = Speed
        self.TTNV = TimeToNearestVehicle
        self.NTI = NextToIntersection
        #self.SOAC = SpeedofAheadCar
       # self.DTAC = DistanceToAheadCar
#        self.relative_distance_speed = RelativeDistanceSpeed
#        self.distance_to_car_of_interest =  DistancetoToCarOfInterest
        self.state= self.TTNV + 4* self.speed + 4*7*self.OnGrid + 4*7*2*self.AdjFreeLane + 4*7*2*3*self.collision +\
                    4*7*2*3*2*self.RLane + 4*7*2*3*2*2*self.Turn + 4*7*2*3*2*2*3*self.Lane+ 4*7*2*3*2*2*3*2*self.Location 
    def Action_training(self, Qtable):
        if (self.OnGrid==1 and self.collision==0):
            if (self.Location==0 or self.Location==2):
                if (self.RLane==1):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.LaneChange = 0  # no lane change when on correct lane and before or after intersection 
                elif (self.RLane==0 and self.AdjFreeLane==0 ):
                    self.LaneChange = 0 # no Lane change when on wrong Lane and adjecent lane is busy
                elif (self.RLane==0 and (self.AdjFreeLane ==1 or self.AdjFreeLane==2) and self.Lane ==1) :
                    self.LaneChange = 1  #right lane change if you are on the wrong  and left lane
                elif (self.RLane==0 and (self.AdjFreeLane ==1 or self.AdjFreeLane==2) and self.Lane ==0) :
                    self.LaneChange = 2  #left lane change if you are on the wrong  and right lane
                else:
                    self.LaneChange = np.array([0,1,2])
                    
            if(self.Location==1 or (self.Location==0 and self.NTI ==1) ):
                if (self.Turn==1):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.MakeTurn = 0  # no turn when no turn is needed and at the intersection 
                elif (self.Turn == 2 and self.Lane==0 ):
                    self.MakeTurn = 1  # turn right when you are in right lane and need to make right
                elif (self.Turn ==0 and self.Lane ==1) :
                    self.MakeTurn = 2  # turn left lane if you are on the left lane and need to make left
                else:
                    self.MakeTurn = np.array([0,1,2])
                      
            if (self.Location==0 or self.Location==2):
                if (self.TTNV == 0 and self.speed >0):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.ChangeSpeed = np.array([0,1,2])  # if the time to nearest vehicle is small and speed greater than zero decrease the speed
                elif (self.TTNV == 0 and self.speed ==0):
                    self.ChangeSpeed = np.array([0,1,2])
                elif (self.TTNV == 3 and self.speed <6):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.ChangeSpeed = 2  # if the time to nearest vehicle is large and speed less than 70 increase the speed
                elif (self.TTNV == 3 and self.speed ==6):
                    self.ChangeSpeed = 0
                else:
                    self.ChangeSpeed = np.array([0,1,2])
                    
            elif(self.Location==1 or (self.Location==0 and self.NTI ==1) ):
                if (self.TTNV == 0 and self.speed >0):       # this can be changed as random later down the road to leave the car to do some manouver 
                    #self.ChangeSpeed = 1  # if the time to nearest vehicle is small and speed greater than zero decrease the speed
                    self.ChangeSpeed = np.array([0,1,2])
                elif (self.TTNV == 0 and self.speed ==0):
                    #self.ChangeSpeed = 0
                    self.ChangeSpeed = np.array([0,1,2])
                elif (self.TTNV == 3 and self.speed <7 and self.Turn==1):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.ChangeSpeed = 2  # if the time to nearest vehicle is large and going stright and speed less than 70 increase the speed
                elif (self.TTNV == 3 and self.speed <5 and (self.Turn==0 or self.Turn==2) ):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.ChangeSpeed = 2  # if the time to nearest vehicle is large and going stright and speed less than 70 increase the speed
                elif (self.TTNV == 3 and self.speed ==5 and (self.Turn==0 or self.Turn==2) ):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.ChangeSpeed = 0  # if the time to nearest vehicle is large and going stright and speed less than 70 increase the speed
                elif (self.TTNV == 3 and self.speed >5 and (self.Turn==0 or self.Turn==2) ):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.ChangeSpeed = 1  # if the time to nearest vehicle is large and going stright and speed less than 70 increase the speed
                elif (self.TTNV == 2 and self.speed <5 and (self.Turn==0 or self.Turn==2) ):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.ChangeSpeed = 2  # if the time to nearest vehicle is large and going stright and speed less than 70 increase the speed
                elif (self.TTNV == 2 and self.speed ==5 and (self.Turn==0 or self.Turn==2) ):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.ChangeSpeed = 0  # if the time to nearest vehicle is large and going stright and speed less than 70 increase the speed
                elif (self.TTNV == 2 and self.speed >5 and (self.Turn==0 or self.Turn==2) ):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.ChangeSpeed = 1  # if the time to nearest vehicle is large and going stright and speed less than 70 increase the speed
                else:
                    self.ChangeSpeed = np.array([0,1,2])

        else:
            self.LaneChange = 0
            self.MakeTurn = 0
            self.ChangeSpeed = 0
        if (self.Location==0 or self.Location==2):    
            if (np.array(self.LaneChange).size>=1 or np.array(self.ChangeSpeed).size >=1):
                x=np.array(self.LaneChange)
                y=np.array(self.ChangeSpeed)
                action_list= np.transpose([np.tile(x, y.size), np.repeat(y, x.size)]) 
                ActionInd=np.random.choice(len(action_list))
                Actionr= action_list[ActionInd,:]
                #ActionpInd=np.argmax(Qtable[self.state,0:])
                ActionpArr = np.ravel(np.where(Qtable[self.state,0:]==np.amax(Qtable[self.state,0:9])))
                ActionpArr = ActionpArr[ActionpArr[:]<9]
                if (len(ActionpArr)>=1):
                   ActionpInd = ActionpArr[0]#np.random.choice(ActionpArr,1) #rrak
                else:
                    ActionpInd =-1
                       
                Actionp = np.zeros(2)
                if (ActionpInd//9==0):
                   # if (np.array(self.LaneChange).size>1):                    
                    Actionp[0] = (ActionpInd)//3 
                    #else:
                    #    Actionp[0] = self.LaneChange
                    Actionp[1] = (ActionpInd)%3
                    set_choose = np.random.choice([0,1],p=[.67,0.33])
                    if (set_choose == 0):
                        self.Action = Actionp
                    else:
                        self.Action = Actionr                    
                else:
    #                    Actionp[0] = 0
    #                    Actionp[1] = 0
    #                    Actionp[1] = (ActionpInd-9)%3
                    self.Action = Actionr
                
                self.LaneChange = self.Action[0]
                self.ChangeSpeed = self.Action[1]
                if (self.Turn==1):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.MakeTurn = 0  # no turn when no turn is needed and at the intersection 
                elif (self.Turn == 2):
                    self.MakeTurn = 1  # turn right when you are in right lane and need to make right
                elif (self.Turn ==0) :
                    self.MakeTurn = 2  # turn left lane if you are on the left lane  turn left
            else:
                if (self.Turn==1):       # this can be changed as random later down the road to leave the car to do some manouver 
                    self.MakeTurn = 0  # no turn when no turn is needed and at the intersection 
                elif (self.Turn == 2):
                    self.MakeTurn = 1  # turn right when you are in right lane and need to make right
                elif (self.Turn ==0) :
                    self.MakeTurn = 2  # turn left lane if you are on the left lane  turn left
            
            self.action= 3* self.LaneChange + self.ChangeSpeed
        elif (self.Location==1):
            if (np.array(self.MakeTurn).size>=1 or np.array(self.ChangeSpeed).size >=1):
                
                x=np.array(self.MakeTurn)
                y=np.array(self.ChangeSpeed)
                action_list= np.transpose([np.tile(x, y.size), np.repeat(y, x.size)]) 
                ActionInd=np.random.choice(len(action_list))
                Actionr= action_list[ActionInd,:]
                #ActionpInd=np.argmax(Qtable[self.state,0:])
                ActionpArr = np.ravel(np.where(Qtable[self.state,0:]==np.amax(Qtable[self.state,9:]))) # take care
                ActionpArr = ActionpArr[ActionpArr[:]>=9]
                if (len(ActionpArr)>=1):
                   ActionpInd = ActionpArr[0]#np.random.choice(ActionpArr,1) #rrak
                else:
                    ActionpInd =-1
                
                Actionp = np.zeros(2)
                if (ActionpInd//9==1):
                    Actionp[0] = (ActionpInd - 9)//3
                    Actionp[1] = (ActionpInd - 9)%3
                    set_choose = np.random.choice([0,1],p=[.67,0.33]) ##rrak
                    if (set_choose == 0):
                        self.Action = Actionp
                    else:
                        self.Action = Actionr
                    
                else:
    #                    Actionp[0] = 0
    #                    Actionp[1] = 0
    #                    Actionp[1] = (ActionpInd)%3
                    self.Action = Actionr
                
                self.MakeTurn = self.Action[0]
                self.ChangeSpeed = self.Action[1]
                self.LaneChange = 0
            else:
                self.LaneChange = 0
            self.action= 9 + 3* self.MakeTurn + self.ChangeSpeed
        else:
            self.LaneChange = 0
            self.ChangeSpeed = 0
            self.MakeTurn = 0
            #Actionp=np.argmax(self.Qtable[self.state,:])
            self.action= 0
            
        self.ac_array = np.array([self.LaneChange, self.ChangeSpeed, self.MakeTurn])    
        return (self.ac_array)       
            
            
        
    def Q_learning_state (self):
        self.old_state= self.TTNV + 4* self.speed + 4*7*self.OnGrid + 4*7*2*self.AdjFreeLane + 4*7*2*3*self.collision +\
                    4*7*2*3*2*self.RLane + 4*7*2*3*2*2*self.Turn + 4*7*2*3*2*2*self.Lane+ 4*7*2*3*2*2*2*self.Location 
#        if iteration == 0:
#            self.Qtable = -1000000*np.ones((9*8*2*6*8*2*4*2*2*3*2*4*2*4*3,3*2*3))
#        else:
#            if (self.Qtable[self.state,self.action]>-1000000):
#                self.Qtable[self.state,self.action] = (1.-0.25) * self.Qtable[self.state,self.action]+ 0.25 * self.TR
#            else:
#                self.Qtable[self.state,self.action]=self.TR
#                
        return self.old_state
        
    def reward(self,iteration):
#        if self.OnGrid == 0:
#            if self.collision > 0:
#                return -1000
#            else:
#                return 0
        if (self.OnGrid == 1):
            self.Rtime = (7. - self.speed)*(-2.0)/7.0
        if (self.RLane == 0 and self.OnGrid == 1):
            self.Rdest = self.Rdest -2
        else:
            self.Rdest = 0
        if (self.OnGrid == 1 and self.TTNV==0):
            self.RTTC = self.RTTC -5
        else:
            self.RTTC = 0 #self.RTTC #the new tt collision to be implemented 
        if (self.collision==1):
            self.Rcoll = -1000
                
        self.TR =self.Rtime + self.Rdest + self.RTTC +self.Rcoll
        
        return self.TR
        
    def reward_def(self):
        
        self.Rtime1 = self.Rtime
        self.Rdest1 = self.Rdest
        if (self.OnGrid == 1 and self.TTNV==0):
            self.RTTC1 = self.RTTC
        else:
            self.RTTC1 = 0
        self.Rcoll1 = 0
        if (self.collision==1):
            self.Rcoll1 = -1000
        self.TR1 =self.Rtime1 + self.Rdest1 + self.RTTC1 +self.Rcoll1
        
        return self.TR1    
        
# location is a tuple to index the Qtable
def updateQvalue(oldval, reward):
#    if (flag==1) :
#        r = 
        
    return .5 * oldval + .5 * reward