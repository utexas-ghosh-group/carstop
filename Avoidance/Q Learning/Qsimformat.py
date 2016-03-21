# -*- coding: utf-8 -*-
"""
This code is intended to handle formatting only, for example:
    which features to exclude
    how to number/discretize certain features
This way, simple changes can be made without changing either Qsim or Agent

+ You can put output formatting (number to tuple) here as well if you want to
"""
import pandas as pd

def discrete(value, bins):
    for k in range(len(bins)):
        if value <= bins[k]:
            return k
    return len(bins)

# state is a Pandas Series
def formatState(state, AgentToRemove = None):
    state = state.copy()

    # check for vehicle removals
    if state['ID'] == AgentToRemove:
        state['On Grid'] = False
        state['Collision'] = False
    if state['Crossing Car'] == AgentToRemove:
        state['Time to Crossing Car'] = state['Time to 2nd Crossing Car']
    if state['Ahead Car'] == AgentToRemove:
        state['Speed of Ahead Car'] = state['Speed of 2nd Ahead Car']
        state['Distance to Ahead Car'] = state['Distance to 2nd Ahead Car']
        state['Time to Ahead Car'] = state['Time to 2nd Ahead Car']
        state['Next to Intersection'] = state['2nd Next to Intersection']
    if state['Behind Car'] == AgentToRemove:
        state['Time to Behind Car'] = state['Time to 2nd Behind Car']
    if state['Colliding Vehicle'] == AgentToRemove:
        state['Collision'] = False
    
    # drop secondary variables
    state = state.drop(['Time to 2nd Crossing Car', 'Crossing Car',
                        '2nd Next to Intersection', 'Ahead Car',
                        'Distance to 2nd Ahead Car', 'Speed of 2nd Ahead Car',
                        'Time to 2nd Ahead Car', 'Time to 2nd Behind Car',
                        'Behind Car', 'Colliding Vehicle'])   
        
    # combine different time-to-collide variables
    timeVals = [state['Time to Crossing Car'], state['Time to Ahead Car']]#,
                    #state['Time to Behind Car']]
    state['Time to Nearest Vehicle'] = min(timeVals)
    #state['Closest Car is Ahead'] = 1
    #if timeVals[2] > timeVals[1] and timeVals[2] > timeVals[0]:
    #    state['Closest Car is Ahead'] = 0
    state = state.drop(['Time to Crossing Car','Time to Ahead Car','Time to Behind Car'])
    
    # choose other variables to drop           
    state = state.drop(['ID', 'Speed of Ahead Car', 'Distance to Ahead Car',
                       'Road', 'Destination Road', 'Destination Lane'])                 
    
    # reformat variables
    state['Speed'] = discrete( state['Speed'] , [0,10,20,30,40,50])
    state['Time to Nearest Vehicle'] = discrete(
                            state['Time to Nearest Vehicle'], [1,2,4])
    
    return state