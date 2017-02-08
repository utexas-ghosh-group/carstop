# -*- coding: utf-8 -*-
"""
The simulated road environment is directly based off of a four-way, two-lane
cross intersection the open source traffic simulator SUMO.
<http://sumo.dlr.de/wiki/Simulation_of_Urban_MObility_-_Wiki>
Instead of interfacing with SUMO, the equations relating road position to 
physical coordinates were handcoded for maximum speed.
"""
import numpy as np

def roadLoc(pos, route):
    if type(pos) == np.ndarray:
        xya = np.empty((pos.shape[0],3))
        if route == 'left-right':
            xya[:,0] = pos
            xya[:,1] = -5.
            xya[:,2] = 0.
        elif route == 'down-up':
            xya[:,0] = 5.
            xya[:,1] = pos
            xya[:,2] = np.pi/2.
        elif route == 'right-down':
            before = pos <= -8.
            after = pos > 7.15818455
            within = (before | after) == False
            xya[before,0] = -pos[before]
            xya[before,1] = 1.65
            xya[before,2] = np.pi
            xya[after,0] = -1.65
            xya[after,1] = -.8418154464 - pos[after]
            xya[after,2] = -np.pi/2.
            r = 9.65
            p = pos[within] + 8.
            xya[within,0] = 8. - r*np.sin(p/r)
            xya[within,1] = -8. + r*np.cos(p/r)
            xya[within,2] = p*.07792350162 - np.sin(p*.3116940065)*.2 - np.pi
        else: raise Exception
        return xya
    else:
        if route == 'left-right':
            return (pos, -5., 0.)
        elif route == 'down-up':
            return (5., pos, np.pi/2.)
        elif route == 'right-down':
            if pos <= -8.:
                return (-pos, 1.65, np.pi)
            elif pos > 7.15818455:
                return (-1.65, -.8418154464 - pos, -np.pi/2.)
            else:
                r = 9.65
                p = pos + 8.
                return (8. - r*np.sin(p/r), -8. + r*np.cos(p/r),
                        p*.07792350162 - np.sin(p*.3116940065)*.2 - np.pi)
        else: raise Exception