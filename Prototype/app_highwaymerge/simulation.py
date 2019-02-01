# -*- coding: utf-8 -*-
"""
last mod 5/18/18
follows format of readSensor()
This code moves fake vehicles along the road,
and keeps a fake merging car at a certain point
"""

import numpy as np
import prototype.app_highwaymerge.road as road
import cv2

#merging_car = np.array([1367.5, 3353.]) # balcones
#merging_car = road.road2Global(206., 2) 
#merging_car = np.array((622335.2, 3361937.5)) # innovation ln
merging_car = road.road2Global(300., 2)
#merging_car = road.road2Global(180., 2) # red river... one of many

# draw background
ppm = 1
grid = (np.min(road.points[:,1])-5, np.max(road.points[:,1])+5,
        np.min(road.points[:,0])-5, np.max(road.points[:,0])+5)
grid = np.array(grid, dtype=int)
im = np.zeros(((grid[1]-grid[0])*ppm, (grid[3]-grid[2])*ppm, 3), dtype=np.uint8) + 255
def coord2img(x,y=None):
    if y==None:
        return int((x[0]-grid[2])*ppm), int((grid[1]-x[1])*ppm)
    else:
        return int((x-grid[2])*ppm), int((grid[1]-y)*ppm)
# draw road
for k in range(road.points.shape[0]-1):
    if road.curve[k]:
        center = coord2img(road.curve_centers[k])
        curve_radius = int(road.curve_mags[k] * ppm)
        nominal_angle = road.curve_start_angles[k] * 180./np.pi
        diff_angle = road.lengths[k] / road.curve_mags[k] * 180./np.pi
        if road.curve_directions[k] < 0: nominal_angle -= diff_angle
        cv2.ellipse(img=im, center=center, axes=(curve_radius, curve_radius),
                    angle=nominal_angle, startAngle=0., endAngle = diff_angle,
                    color=[160,160,160], thickness=12)
    else:
        cv2.line(im, coord2img(road.points[k]),
             coord2img(road.points[k+1]), color = [160,160,160], thickness=12)
# draw merging vehicle
cv2.circle(im, coord2img(merging_car), radius = 3, color = [230,150,100],
           thickness = 4)
#cv2.imshow('Random Test', im)
#cv2.waitKey(1)
#cv2.destroyWindow('Random Test')

min_speed = 4
max_speed = 8
max_time_between = 45.
cars = []

# put mutable parameters into a dictionary for easy function referencing
stuff = {'time' : 0, 'next speed' : 4., 'next time' : 0}

def readSensor():
    if stuff['time'] == stuff['next time']:
        cars.append([0., stuff['next speed']])
        next_speed = np.random.uniform(min_speed, max_speed)
        min_time_between = max(3,
                (road.sumlengths[-1]/stuff['next speed'] - road.sumlengths[-1]/next_speed))
        time_between = np.random.uniform(min_time_between, max_time_between)
        stuff['next time'] = int(stuff['time'] + time_between)
        stuff['next speed'] = next_speed
    stuff['time'] += 1
        
    img = im.copy()
    car_coords = []
    alt_return = []
    for car_idx, car in enumerate(cars):
        if car[0] > road.sumlengths[-1]:
            continue
        car_coord = road.road2Global(car[0])
        car_coords.append(car_coord)
        carp = coord2img(car_coord)
        cv2.circle(img, carp, radius=3, color=[100,150,230], thickness=4)
        # move cars
        car[0] += car[1]
        
        alt_return += [(car_coord, car[1]*5, car_idx)]
        
    cv2.imshow('Random Test', img)
    key = cv2.waitKey(200)
    if key == -1 or key == 255: # no key pressed
        pass
    elif 'p' == chr(key & 255): # pause
        key = cv2.waitKey(60000)
    else: return None # end program if other key pressed
    
    #return merging_car, np.array(car_coords)
    me_return = (merging_car, 0.)
    return me_return, alt_return
    
def startSensor(): pass
def stopSensor(): cv2.destroyWindow('Random Test')

if __name__ == '__main__':
    try:
        while stuff['time'] < 100:
            readSensor()
    finally:
        stopSensor()
