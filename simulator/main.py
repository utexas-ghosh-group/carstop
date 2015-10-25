#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2015 sean <sean@wireless-10-145-81-36.public.utexas.edu>
#
# Distributed under terms of the MIT license.
"""TODO(sean): DO NOT SUBMIT without one-line documentation for test

TODO(sean): DO NOT SUBMIT without a detailed description of test.
"""
import argparse
import os
import re
import sys
import time
import traceback

from world import *

def main():
  global args
  world = World(500, 500)
  road = Road(Point(0, world.GetWidth() / 2), Point(world.GetWidth(),
    world.GetHeight() / 2), 100)
  world.AddRoad(road)
  car = Car(position=Point(100, world.GetHeight() / 2))
  world.AddCar(car)
  world.Simulate()

if __name__ == '__main__':
  try:
    start_time = time.time()
    parser = argparse.ArgumentParser()
    parser.add_argument('-v','--verbose', action='store_true', default=False, \
        help='verbose output')
    parser.add_argument('-d','--debug', action='store_true', default=False, \
        help='debug output')
    args = parser.parse_args()
    # if len(args) < 1:
    #   parser.error('missing argument')
    if args.verbose: print(time.asctime())
    main()
    if args.verbose: print(time.asctime())
    if args.verbose: print('TOTAL TIME IN MINUTES:')
    if args.verbose: print(time.time() - start_time) / 60.0
    sys.exit(0)
  except KeyboardInterrupt, e: # Ctrl-C
    raise e
  except SystemExit, e: # sys.exit()
    raise e
  except Exception, e:
    print('ERROR, UNEXPECTED EXCEPTION')
    print(str(e))
    traceback.print_exc()
    os._exit(1)
