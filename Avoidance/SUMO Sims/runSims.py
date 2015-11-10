# -*- coding: utf-8 -*-
"""
Example to run multiple (of the same) simulation and store .csv results.
last modified 11/8/15
"""

import os, sys, runSim
from constants import *  ## need tools path

numberOfRuns = 30
nameOfSim = "highway"

# for calling from command line, ex. python runSims.py 10
if len(sys.argv) == 2:
    numberOfRuns = int(sys.argv[1])

for run in range(1,numberOfRuns+1):
    runSim.init(run)
    thisSumoOutFile = "./Results/out" + str(run) + ".xml"
    thisCsvFile = "./Results/" + nameOfSim + str(run) + ".csv"
    os.system("python ../tools/xml/xml2csv.py " + thisSumoOutFile + 
        " --output " + thisCsvFile)