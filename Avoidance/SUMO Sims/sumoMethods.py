# -*- coding: utf-8 -*-
"""
Contains methods to create, end, and control SUMO simulation.
last major edit 11/17/15
edit 12/8/15 : changed subprocess output
"""
import subprocess, sys, os
#toolsPathName = "../../../cars/sumo/tools"
toolsPathName = "/Users/twankim/carstop/sumo-0.24.0/tools"
#binPathName = "../../../cars/sumo/bin"
sys.path.append(os.path.realpath(toolsPathName))
import signal
#sys.path.append(os.path.realpath(binPathName))
# above line is necessary if you can't fully install sumo

_wait_in_s = 6

import traci

class Sumo:
    
    def __init__(self, configFile, gui = False, outFile=None):
        """ starts the simulation
            inputs:
            configFile = string, name of the configuration you're using
            gui = boolean (optional), if True, the SUMO gui will pop up
            outFile = string (optional), file for SUMO to save data
        """
        PORT = 8800             
        
        # from sumo demo code (vehicleControl.py)
        try:
            from sumolib import checkBinary
        except ImportError:
            def checkBinary(name):
                return name 
        
        # write terminal command
        completeCommand = [checkBinary("sumo")]
        if gui:
            completeCommand = [checkBinary("sumo-gui")]
        sumoConfig = "%s.sumocfg" % (configFile)
        completeCommand += ["-c", sumoConfig]
        if outFile is not None:
            completeCommand += ["--fcd-output","./Results/%s.xml" % outFile ]
        
        ## start SUMO
        self.sumoProcess = subprocess.Popen(completeCommand,
                                            stdout=subprocess.PIPE,#sys.stdout,#
                                            stderr=sys.stderr)
        traci.init(PORT, 10)
        
        self.outFile = outFile
    
    def createVehicle(self,vehID, laneID, pos=0):
        signal.signal(signal.SIGALRM, _timeOutHandler)
        signal.alarm(_wait_in_s)
        try:
            self._createVehicle(vehID, laneID, pos)
        except Exception, exc:
            print exc
            signal.alarm(_wait_in_s)
            try:
                 self._createVehicle(vehID, laneID, pos)
            except Exception, exc:
                print exc
                print "  quitting on instruction create "+vehID
                self.end()
                return 1 # Sim has failed
        signal.alarm(0)
        return 0
    
    def _createVehicle(self,vehID, laneID, pos=0):
        """ inputs:
            vehID = string
            laneID = string
            pos = numeric (default 0)
            
            for this to work as currently implemented, all lanes in the .net
            file must be named 'road_laneIndex'
            and there must be a route (.rou file) named 'road'
        """
        routeID = laneID[:laneID.rfind('_')]
        laneIndex = int( laneID[laneID.rfind('_')+1:] )
        traci.vehicle.add(vehID,routeID,pos=pos,lane=laneIndex)
        
        traci.simulationStep()
        
    def moveVehicleAlong(self, vehID, dist, lane=None):
        signal.signal(signal.SIGALRM, _timeOutHandler)
        signal.alarm(_wait_in_s)
        try:
            self._moveVehicleAlong(vehID, dist, lane)
        except Exception, exc:
            print exc
            signal.alarm(_wait_in_s)
            try:
                 self._moveVehicleAlong(vehID, dist, lane)
            except Exception, exc:
                print exc
                print "  quitting on instruction move "+vehID+" "+str(dist)
                self.end()
                return 1 # sim has failed
        signal.alarm(0)
        return 0
        
    def _moveVehicleAlong(self,vehID, dist, lane=None):
        """ inputs:
            vehID = string
            dist = numeric, how far the vehicle should travel in meters
            lane = string (optional), lane to switch to
            
            knowing when it's appropriate to switch roads is up to the
            user for now
        """
        prevLane = traci.vehicle.getLaneID(vehID)
        prevPos = traci.vehicle.getLanePosition(vehID)
        
        if lane is None: # assume you can remain on current lane
            lane = prevLane           
            
        if lane is prevLane: # staying in same lane
            currPos = prevPos + dist
        else: # changing lanes, assume you complete old lane
            currPos = prevPos + dist - traci.lane.getLength(lane)
        
        traci.vehicle.moveTo(vehID, lane, currPos)
        traci.vehicle.setSpeed(vehID,0)
        traci.simulationStep()
        
    def getVehicleState(self,vehID):
        """ input: vehID = string
            output: [laneID = string, position on lane = num,
                     coordinate position = (num,num)]
        """
        # [lane id, position along lane, position as (x,y)]
        return [traci.vehicle.getLaneID(vehID),
                traci.vehicle.getLanePosition(vehID),
                traci.vehicle.getPosition(vehID)]
        
    def getLaneInfo(self,lane):
        """ input: laneID = string
            output: [lane length = num, lane width = num,
                     IDs of upcoming connected lanes = [string] ]
        """
        # traci returns extra information, as below...
        ##getLinks(string) -> list((string, bool, bool, bool))
        ##A list containing ids of successor lanes together with
        ##priority, open and foe.
        
        successorlinks = traci.lane.getLinks(lane)
        linknames = [a[0] for a in successorlinks]
        return [traci.lane.getLength(lane),
                traci.lane.getWidth(lane),
                linknames]
    
    def end(self):
        signal.signal(signal.SIGALRM, _timeOutHandler)
        signal.alarm(_wait_in_s)
        try:
            traci.close()
            self.sumoProcess.wait()
        except Exception, exc:
            print exc
            self.sumoProcess.terminate()
            traci._connections[""].close()
            del traci._connections[""]
        signal.alarm(0)
            
        if self.outFile is not None: # transport results to csv
            thisSumoOutFile = "./Results/" + self.outFile + ".xml"
            thisCsvFile = "./Results/" + self.outFile + ".csv"
            os.system("python " + toolsPathName + "/xml/xml2csv.py " +
                        thisSumoOutFile + " --output " + thisCsvFile)
                        
def _timeOutHandler(signum, frame):
    raise Exception("SUMO didn't respond")