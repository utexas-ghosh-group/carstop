# -*- coding: utf-8 -*-
"""
File with common tests for controllers.  Should make it easier to debug
without running SUMO.
last modified 6/16/11
"""

import math
import sys, os
sys.path.append(os.path.dirname(__file__)[:-len("/Controllers")])
from constants import VState
import __init__

vehiclestate = [4.0, 2.0, 2.0, 2.0]
subject = __init__.RearEndBrake("me",vehiclestate, 0.5)
subject.updateSpeed(25.0)

## test 1 - slow vehicle in front, collision in one second
car1 = VState("a", 0.0, -9.0, 0.0, 5.0, 4.0, 2.0, 2.0, 2.0)
# displays the recommended speed change, from 10 m/s
# with default settings, minimum speed for next turn is 9
# and max is 11
print "test 1",subject.nextStep([car1])

## test 2 - faster vehicle in front
car1 = VState("a", 0.0, -5.0, 0.0, 10.5, 4.0, 2.0, 2.0, 2.0)
print "test 2",subject.nextStep([car1])

## test 3 - perpendicular, both hit fronts
car1 = VState("a", -6.0, -11.0, math.pi/2, 5.0, 4.0, 2.0, 2.0, 2.0)
print "test 3",subject.nextStep([car1])

## test 4 - perpendicular, you hit their back
car1 = VState("a", -1.0, -11.0, math.pi/2, 5.0, 4.0, 2.0, 2.0, 2.0)
print "test 4",subject.nextStep([car1])

## test 5 - perpendicular, they hit your back
car1 = VState("a", -6.0, -6.0, math.pi/2, 5.0, 4.0, 2.0, 2.0, 2.0)
print "test 5",subject.nextStep([car1])