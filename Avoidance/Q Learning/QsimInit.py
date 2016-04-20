# -*- coding: utf-8 -*-
"""
Eventually we can set up more complex simulations from this file.
"""
import pandas as pd
import numpy as np 
# takes an iteration number
# return a pandas dataframe with ['time', 'lane', 'dest', 'speed'  ]
# 'time' = creation time for each vehicle, in .1s resolution
# 'lane' = exact lane the vehicle starts in, format 'Ai_B'
#           A is road [1,2,3,4]         B is lane [0,1]
# 'dest' = exact lane the vehicle wants to end in, format 'Ao_B'
# 'speed' vehicle's starting speed in meters per second
def initialize(iteration):
    
#    SourceList =  np.array(['1i_0','1i_1','2i_0','2i_1'])
#    DestList = np.array(['1o_0','1o_1','2o_0','2o_1','3o_0','3o_1','4o_0','4o_1'])
#    CarCount = 5
#    CarInit= pd.DataFrame(np.zeros([CarCount,4]),columns = ['time','lane','dest','speed'])
#    CarInit['lane'] = CarInit['lane'].astype(str)
#    CarInit.loc[0]= [0.0,'1i_1','4o_1',8.]
#    CarInit.loc[1]= [0.0,'2i_0','1o_0',8.]
#    CarInit.loc[2]= [1.0,'1i_1','4o_1',8.]
#    CarInit.loc[3]= [2.0,'2i_0','1o_0',8.]
#    CarInit.loc[4]= [4.0,'2i_0','1o_0',8.]
#    return CarInit
    
    meanCarNum = (iteration)**.5
    SourceList =  np.array(['1i_0','1i_1','2i_0','2i_1'])#,'3i_0',
                                #'3i_1','4i_0','4i_1'])
    DestList = np.array(['1o_0','1o_1','2o_0','2o_1','3o_0','3o_1','4o_0','4o_1'])
    CarCount = np.random.poisson(meanCarNum, 1) + 1
    CarInit= pd.DataFrame(np.zeros([CarCount,4]),columns = ['time','lane','dest','speed'])
    CarInit['lane'] = CarInit['lane'].astype(str)
    sd =np.zeros([2]).astype(int)
    for i in range (CarCount):
        sd[0] = np.random.randint(0,len(SourceList),1)
        sd[1] = np.random.randint(0,len(DestList),1)
        while sd[0]//2 == sd[1]//2:
            sd = np.random.randint(0,min(len(SourceList),len(DestList)),2)
        CarSpeed = np.random.randint(1,4)*10.0/3.6
        s = SourceList[sd[0]]
        d = DestList[sd[1]]
        if np.shape(CarInit[CarInit['lane'].str.contains(s)])[0]>=1:
            CarTime = (np.shape(CarInit[CarInit['lane'].str.contains(s)])[0])*2.
        else:
            CarTime = 0
        CarTime = CarTime + np.random.uniform(0,.5)
        CarInit.loc[i]= [CarTime,s,d,CarSpeed]
    return CarInit