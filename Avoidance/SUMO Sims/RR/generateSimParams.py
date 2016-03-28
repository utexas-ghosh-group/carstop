# -*- coding: utf-8 -*-
"""
3/23/16
"""
import random
from math import sqrt
import pandas as pd

PDIG = 3       ## number of decimal digits to store
MPH2MS = .447  ## miles per hour to meters per second conversion
KPH2MS = 1/3.6 ## kilos per hour to meters per second

trackLength = 1300 # meters
vehicleLength = 5.8 # meters

""" Here are the simulation parameters """
def perceptionResponseTime():
    return random.triangular(1,4)
    
def velocityLead():
    return truncatedNormal(70,10,55,90) * MPH2MS

def velocityOncoming():
    return truncatedNormal(70,10,55,90) * MPH2MS
    
def velocityPassing(velocity_lead):
    vLeadInMPH = velocity_lead / MPH2MS;
    return truncatedNormal(max(vLeadInMPH,70),10,
                            vLeadInMPH - 10, 90) * MPH2MS

def accelPassing():
    return 0.0

def accelDuringOvertake():   
    return truncatedNormal(4.0, 2.0, 1.1, 9.0) * KPH2MS

def accelLead(accel_overtake):
    return truncatedNormal(0, .3, -1.0, min(1.0,accel_overtake))

def accelOncoming():
    return truncatedNormal(0, .3, -1.0, 1.0)
    
def positionPassing():
    return vehicleLength + 20

def positionLead(position_passing, velocity_lead, velocity_passing, accel_lead,
                 accel_passing):
    # one second headway
    headway = velocity_passing + accel_passing/2 + vehicleLength
    position = headway + truncatedNormal(0, 4, -5, 5)
    position = position_passing + position
    return position

def positionOncoming(trp, position_passing, position_lead, velocity_passing,
                     velocity_lead, velocity_oncoming,
                     accel_lead, accel_oncoming):
    # should be sufficiently far from leader at start of overtake
    leadPosAtTrp = position_lead + velocity_lead*trp + accel_lead/2*trp*trp
    minOncomingPos = leadPosAtTrp + velocity_oncoming*trp + accel_oncoming/2*trp*trp
    # this is a (lowered) estimate of the necessary distance from a max-
    # accelerating passing vehicle to make it - with 0 accel for other cars
    lowDist = sqrt(velocity_lead)*(velocity_oncoming + velocity_lead)
    minOncomingPos = minOncomingPos + lowDist
    # recommended sight distance from paper
    maxOncomingPos = (position_passing + .35*velocity_passing*velocity_passing
                    + 15*velocity_passing)
    maxOncomingPos = min(maxOncomingPos, trackLength - vehicleLength)
    return random.uniform(minOncomingPos, maxOncomingPos)

#def moveXBack300ms(position, velocity, accel): # needed for Veins
#    return position - velocity*.3 - accel*.045
#
#def moveVBack300ms(velocity, accel):
#    return velocity - accel*.3
def moveXBack(position, velocity, accel, dt):
    return position - velocity*dt - accel*dt*dt/2.    
    
def moveVBack(velocity, accel, dt):
    return velocity - accel*dt

""" End of simulation parameters """


def truncatedNormal(mean, sd, minvalue = 0, maxvalue = 30):
    answer = minvalue - 1
    counter = 10000
    while (answer < minvalue or answer > maxvalue) and counter > 0:
        answer = random.gauss(mean, sd)
        counter += -1
    if counter <= 0:   # failed to sample
        return (minvalue + maxvalue) / 2
    return answer



class Output:
    def __init__(self, colnames):
        self.df = None
        self.colnames = colnames
    def add(self, newrow):
        if self.df is None:
            self.df = pd.DataFrame([newrow])
        else:
            self.df = self.df.append([newrow])
    def write(self, fileName, restart=False):
        self.df.columns =  self.colnames
        self.df.to_csv(fileName, sep=',', header=True, index=False)
        if restart:
            self.df = None

## main code here
if __name__ == "__main__":
    numiter = 2000
 
    forSims = Output(['tpr','Passing Position','Lead Position','Oncoming Position',
                      'Passing Speed','Lead Speed','Oncoming Speed',
                      'Passing Acceleration','Lead Acceleration',
                      'Oncoming Acceleration'])
    forResults = Output(['Perception Response Time','Lead Vehicle Distance',
                      'Oncoming Vehicle Distance','Passing Vehicle Speed',
                      'Lead Vehicle Speed','Oncoming Vehicle Speed',
                      'Overtaking Acceleration','Lead Vehicle Acceleration',
                      'Oncoming Vehicle Acceleration'])
    
#    random.seed()
#    seed = random.randint(1,1000000000)
#    if len(sys.argv) == 4:
#        seed = int(sys.argv[3])
#    random.seed(seed)
    for iteration in range(numiter):
        tpr = perceptionResponseTime()
        vLead = velocityLead()
        vOncoming = velocityOncoming()
        vPassing = velocityPassing(vLead)
        aPassing = accelPassing()
        aOvertake = accelDuringOvertake()
        aLead = accelLead(aOvertake)
        aOncoming = accelOncoming()
        xPassing = positionPassing()
        xLead = positionLead(xPassing,vLead, vPassing, aLead, aPassing)
        xOncoming = positionOncoming(tpr, xPassing, xLead, vPassing, vLead,
                                     vOncoming, aLead, aOncoming)
        
        forSims.add([tpr, xPassing, xLead, trackLength - xOncoming, vPassing,
                     vLead, vOncoming, aOvertake, aLead, aOncoming])
        forResults.add([tpr, xLead - xPassing, xOncoming - xPassing, vPassing,
                        vLead, vOncoming, aOvertake, aLead, aOncoming])
                    
    forSims.write('simParameters.csv')
    forResults.write('../Parameters/RR_Parameters.csv')