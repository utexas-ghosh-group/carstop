# -*- coding: utf-8 -*-
"""
last mod 11/23/2018
simulated overtaking maneuver, car is driving behind lead car
"""

road_angle = .5 # radians
car_speeds = 30. # m/s
timestep = .5 # s
lead_headway = 2. # s
oncoming_car_rate = .2 # 1/s, poisson process
img_size = 640 # pixels, square
img_view = 1000 # meters, square


import cv2
import numpy as np

car_mvmt = car_speeds * timestep
oncoming_car_rate *= timestep
img_mid = img_size // 2
road_width = 4. # meters per lane
road_color = [180,180,180]
car_size_pixels = 4
dist_of_img_center = img_view // 2 - 20 # meters, distance ahead of car
ppm = float(img_size)/img_view # pixels per meter

csp = car_size_pixels
circle_x, circle_y = np.meshgrid(np.arange(-csp,csp+1, dtype=float),
                                 np.arange(-csp,csp+1, dtype=float))
circle = np.hypot(circle_x, circle_y) <= 3.00001
def drawCircle(image, x, y, color):
    if x < csp or x >= img_size-csp: return
    if y < csp or y >= img_size-csp: return
    x = int(x)
    y = int(y)
    subset = image[x-csp:x+csp+1, y-csp:y+csp+1].copy()
    subset[circle] = color
    image[x-csp:x+csp+1, y-csp:y+csp+1] = subset

image = np.zeros((img_size, img_size, 3), dtype=np.uint8) + 255
road_cos = np.cos(road_angle)
road_sin = np.sin(road_angle)
road_width_y = -int(road_width*road_cos*img_size/img_view)
road_width_x = int(road_width*road_sin*img_size/img_view)
# draw road
for x in xrange(img_size):
    y = img_mid - int((img_mid-x)*road_sin/road_cos)
    rwy = abs(road_width_y)
    image[x, max(0, y-rwy):max(0, y+rwy)] = road_color
car_coord = 0
# draw your car
img_center = (dist_of_img_center * road_cos - road_width/2*road_sin,
              dist_of_img_center * road_sin + road_width/2*road_cos)
x = 0
y = 0
x = (x - img_center[0])*ppm + img_mid
y = (y - img_center[1])*ppm + img_mid
drawCircle(image, x, y, [255,0,0])


class SimSensor():
    def __init__(self, port=None): pass
        
    def __enter__(self):
        cv2.imshow('sim', image)
        self.my_coord = (0., 0.)
        self.oncoming_cars = []
        self.oncoming_count = 1
        return self
    
    def __exit__(self, errtype, errval, traceback):
        cv2.destroyWindow('sim')
    
    def get(self):
        image2 = image.copy()
        
        my_speed = np.array((car_speeds*road_cos, car_speeds*road_sin))
        oncoming_speed = my_speed * -1
        
        my_coord = (self.my_coord[0]+car_mvmt*road_cos,
                    self.my_coord[1]+car_mvmt*road_sin)
        self.my_coord = my_coord
        
        # move lead car
        lead_coord = (my_coord[0] + car_mvmt*lead_headway*road_cos,
                      my_coord[1] + car_mvmt*lead_headway*road_sin)
        returned_cars = [(np.array(lead_coord), my_speed.copy(), 0)]
        
        my_coord_other_lane = (my_coord[0] - road_width*road_sin,
                               my_coord[1] + road_width*road_cos)
        
        # move oncoming cars
        new_oncoming_cars = []
        for car_id, dist in self.oncoming_cars:
            dist -= car_mvmt * 2 # you move, they move
            if dist < -30: continue # has passed you by
            new_oncoming_cars.append((car_id, dist))
            
            coord = (my_coord_other_lane[0] + dist*road_cos,
                     my_coord_other_lane[1] + dist*road_sin)
            returned_cars.append((np.array(coord), oncoming_speed, car_id))
         
        # add new oncoming car
        if np.random.rand() < oncoming_car_rate:
            car_id = self.oncoming_count
            self.oncoming_count += 1
            new_oncoming_cars.append((car_id, 1300))
            coord = (my_coord_other_lane[0] + 1300*road_cos,
                     my_coord_other_lane[1] + 1300*road_sin)
            returned_cars.append((np.array(coord), oncoming_speed, car_id))
        self.oncoming_cars = new_oncoming_cars
        
        # draw cars
        for coord, speed, car_id in returned_cars:
            # convert coordinates to pixels
            coord = (coord[0] - my_coord[0], coord[1] - my_coord[1])
            x = (coord[0] - img_center[0])*ppm + img_mid
            y = (coord[1] - img_center[1])*ppm + img_mid
            drawCircle(image2, x, y, [0,0,255])
        
    
        cv2.imshow('sim', image2)
        cv2.waitKey(int(timestep*1000))
        return (my_coord, my_speed), returned_cars
    
if __name__ == '__main__':
    with SimSensor() as s:
        while True:
            s.get()