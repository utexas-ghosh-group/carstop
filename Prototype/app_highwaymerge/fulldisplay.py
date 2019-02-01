# -*- coding: utf-8 -*-
"""
last mod 6/25/2018 error in handling cv2.ellipse
this code is a big mess that uses OpenCv drawing functions to make a decent GUI
"""

import numpy as np
import cv2
import road
import warnGUI
#from skvideo.io import FFmpegWriter as vwriter

dist_ahead = 20. # meters on road
dist_behind = 120.
dist_around = 15.
pix_size = (600,240)
road_thickness = 6 # meters
arrow_size = 3. # meters
pix_buffer = 0 # pixels to either side, prevents wonky stuffz
warning_sign_pos = (25,75,225,275)
compass_pos = (10,50,450,490)
car_len = 24 # pixels
dist_to_change_gui = 15.

background_color = [230,255,240]
road_color = [160,160,160]
you_car_color = [230,150,100]
alt_car_color = [100,150,230]
merge_arrow_color = [0,0,0]

def rotationMatrix(angle):
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle),  np.cos(angle)]])
    
def coord2Img(coord, center, ppm, imshape):
    #return max(0, min(imshape[1]-1,
    #                  int((center[1]-coord[1])*ppm + imshape[1]/2))) ,\
    #       max(0, min(imshape[0]-1,
    #                  int((coord[0]-center[0])*ppm + imshape[0]/2)))
    return int((center[1]-coord[1])*ppm + imshape[1]//2) ,\
           int((coord[0]-center[0])*ppm + imshape[0]//2)
      

my_folder = __file__[:-15] if __file__[-4:]=='.pyc' else __file__[:-14]
# load warning sign
wsign = cv2.imread(my_folder+'Attention_Sign.svg.png')
# shrink by subsampling
newsignshape = ( warning_sign_pos[1]-warning_sign_pos[0] ,
                 warning_sign_pos[3]-warning_sign_pos[2] )
wsign = cv2.resize(wsign, newsignshape)
#hindexes = np.linspace(0, wsign.shape[0], newsignshape[0], endpoint=False, dtype=int)
#windexes = np.linspace(0, wsign.shape[1], newsignshape[1], endpoint=False, dtype=int)
#wsign = wsign[hindexes,:,:][:,windexes,:]
# mask
wsign_included = np.any(wsign > 10, axis=2)[:,:,None]

# make compass rose
compass_shape = compass_pos[1]-compass_pos[0]
assert compass_shape == compass_pos[3]-compass_pos[2] # must be circle
#compass = np.zeros((compass_shape, compass_shape, 3), dtype=np.uint8) + 255
def addCompass(compass_im, rotmat):
    assert compass_im.shape[0] == compass_shape
    assert compass_im.shape[1] == compass_shape
    halfshape = compass_shape//2
    cv2.circle(compass_im, (halfshape, halfshape), halfshape,
               [190,190,190], thickness=-1, lineType=cv2.LINE_AA)
    north_triangle = [[-compass_shape//4,0], [compass_shape//4,0], [0,-halfshape+1]]
    north_triangle = np.array(north_triangle).dot(rotmat).astype(np.int32)
    south_triangle = -north_triangle
    north_triangle += halfshape
    south_triangle += halfshape
    north_triangle = north_triangle[None,:,:].copy()
    south_triangle = south_triangle[None,:,:].copy()
    cv2.fillConvexPoly(compass_im, north_triangle, [0,0,255], lineType=cv2.LINE_AA)
    cv2.fillConvexPoly(compass_im, south_triangle, [255,255,255], lineType=cv2.LINE_AA)
    
# load car icon
carimg = cv2.imread(my_folder+'greencar.png')
car_wid = int(carimg.shape[0]*car_len/carimg.shape[1]) // 2 * 2
carimg = cv2.resize(carimg, (car_len, car_wid))
car_mask = np.any(carimg > 1, axis=2)
color_parts = np.any(carimg > 50, axis=2)
you_img = carimg.copy()
you_img[color_parts] = you_car_color
you_img = you_img.transpose((1,0,2))[::-1,:,:] # face up
you_img_mask = car_mask.transpose((1,0))[::-1,:]
alt_img = np.zeros((car_len, car_len, 3), dtype=np.uint8)
alt_img[car_len//2-car_wid//2 : car_len//2+car_wid//2] = carimg
alt_img[car_len//2-car_wid//2 : car_len//2+car_wid//2][color_parts] = alt_car_color
def rotateSquareImg(img, angle):
    M = cv2.getRotationMatrix2D((img.shape[0]//2, img.shape[1]//2), angle*180/np.pi, 1.)
    return cv2.warpAffine(img, M, img.shape[1::-1])

def addSubImage(img, sub_img, sub_mask, point):
    minh = point[0] - sub_img.shape[0]//2
    maxh = point[0] + sub_img.shape[0]//2
    minw = point[1] - sub_img.shape[1]//2
    maxw = point[1] + sub_img.shape[1]//2
    subminh = 0
    submaxh = sub_img.shape[0]
    subminw = 0
    submaxw = sub_img.shape[1]
    if minh < 0:
        subminh = -minh
        minh = 0
    if maxh > img.shape[0]:
        submaxh += img.shape[0]-maxh
        maxh = img.shape[0]
    if minw < 0:
        subminw = -minw
        minw = 0
    if maxw > img.shape[1]:
        submaxw = img.shape[1]-maxw
        maxw = img.shape[1]
    if maxh <= minh or maxw <= minw: return
    masked_img = np.where(sub_mask[subminh:submaxh, subminw:submaxw, None],
                    sub_img[subminh:submaxh, subminw:submaxw], img[minh:maxh, minw:maxw])           
    img[minh:maxh, minw:maxw] = masked_img

# arrow to signify merging
arrow_points = [[1,-3],[1,-1],[4,-1],[4,-2],[6,0],[4,2],[4,1],[-1,1],[-1,-3]]
arrow_points = np.array(arrow_points, dtype=float) * [1,-1]



class Display():
    def __init__(self, merging_car, restarting=False):
        self.current_merging_car = merging_car.copy()
        merging_pos, merging_lane = road.global2Road(merging_car)
        assert not (merging_lane is None) and merging_lane > 0
        
        # find angle of merge and other road values
        link = np.searchsorted(road.sumlengths[1:], merging_pos)
        assert link < road.nroads
        if road.curve[link]:
            angle = road.curve_start_angles[link]
            angle += (merging_pos - road.sumlengths[link]) *\
                        road.curve_directions[link]/road.curve_mags[link]
        else:
            angle = np.arctan2(road.along_vec[link][1], road.along_vec[link][0])
        rotmat = np.linalg.inv(rotationMatrix(angle)) # rotate global points to merge POV
        center = road.road2Global(merging_pos, 0)
        #center = merging_pos
        ahead_point = road.road2Global(merging_pos + dist_ahead, 0)
        behind_point = road.road2Global(max(0., merging_pos - dist_behind), 0)

        self.rotmat = rotmat
        self.ref_center = center
        
        # determine size of grid
        bounding_points = np.array([self.reference(ahead_point),
                                    self.reference(behind_point),
                                    [-dist_around, 0],
                                    [dist_around, 0],
                                    [0, -dist_around],
                                    [0, dist_around]])
        im_center = np.max(bounding_points, axis=0)//2 + np.min(bounding_points, axis=0)//2
        bound_sizes = np.max(bounding_points, axis=0) - np.min(bounding_points, axis=0)
        ppm_axes = pix_size / bound_sizes
        bounding_axis = np.argmin(ppm_axes)
        ppm = ppm_axes[bounding_axis]
        assert ppm > .25
        assert ppm < 10
        
        self.im_center = im_center
        self.ppm = ppm
        
        # plot nearby roads
        im = np.zeros((pix_size[1], pix_size[0], 3),
                      dtype=np.uint8)
        im[:] = background_color
        self.imshape = pix_size
        road_thickness_img = int(road_thickness * ppm)
        for k in range(road.points.shape[0]-1):
            if road.curve[k]:
                roadcenter = self.reference(road.curve_centers[k])
                roadcenter = self.convert2Img(roadcenter)
                curve_radius = int(road.curve_mags[k] * ppm)
                nominal_angle = road.curve_start_angles[k] - angle
                nominal_angle = -nominal_angle * 180./np.pi
                diff_angle = road.lengths[k] / road.curve_mags[k] * 180./np.pi
                if road.curve_directions[k] > 0: nominal_angle += diff_angle
                cv2.ellipse(img=im, center=roadcenter, axes=(curve_radius, curve_radius),
                            angle=nominal_angle, startAngle=0., endAngle = diff_angle,
                            color=road_color, thickness=road_thickness_img,
                            lineType=cv2.LINE_AA)
            else:
                start = self.reference(road.points[k])
                start = self.convert2Img(start)
                end = self.reference(road.points[k+1])
                end = self.convert2Img(end)
                cv2.line(im, start, end, color = road_color,
                         thickness=road_thickness_img, lineType=cv2.LINE_AA)

        # draw arrow to signify merging
        merge_in_img = self.convert2Img([0,0])
        these_arrow_points = (arrow_points[None,:,:]*ppm + merge_in_img).astype(np.int32)
        cv2.polylines(img=im, pts=these_arrow_points, isClosed=True,
                      color=merge_arrow_color, thickness = 2, lineType=cv2.LINE_AA)
        
        # draw compass
        addCompass(im[compass_pos[0]:compass_pos[1], compass_pos[2]:compass_pos[3]],
                   rotmat)
        
        self.im = im
        
        if not restarting:
            self.audio = warnGUI.GUI(video=False)
            self.audio.startDisplay()
            #self.vid = vwriter('recorded demo.mp4', outputdict={'-y':'-an'})
        
    def reference(self, coord):
        return self.rotmat.dot(coord - self.ref_center)
        
    def convert2Img(self, coord):
        return coord2Img(coord, self.im_center, self.ppm, self.imshape)[::-1]
        
    def changeDisplay(self, display_command, you_coord, alt_cars):
        if display_command == 4: # close command
            self.stopDisplay()
            return
        
        # if the distance between your current gui and the new point is too high,
        # readjust the whole display
        if np.hypot(self.current_merging_car[0] - you_coord[0],
                    self.current_merging_car[1] - you_coord[1]) > dist_to_change_gui:
            self.__init__(you_coord, restarting = True)
        img = self.im.copy()
        
        # draw merging location
        merging_car_rel = self.convert2Img(self.reference(you_coord))
        addSubImage(img, you_img, you_img_mask, merging_car_rel[::-1])
        #cv2.circle(img, merging_car_rel,
        #           radius = 4, color = [230,150,100], thickness = -1)
        # draw other cars
        for alt_pos, alt_lane, alt_speed, alt_id in alt_cars:
            alt_coord = road.road2Global(alt_pos, alt_lane)
            alt_coord = self.convert2Img(self.reference(alt_coord))
            link = np.searchsorted(road.sumlengths[1:], alt_pos)
            if link > road.nroads: continue
            alt_angle = self.rotmat.dot(road.along_vec[link])
            alt_angle = np.arctan2(alt_angle[1], alt_angle[0])
            alt_img_rotated = rotateSquareImg(alt_img, alt_angle)
            alt_img_mask_rotated = np.any(alt_img_rotated > 1, axis=2)
            addSubImage(img, alt_img_rotated, alt_img_mask_rotated, alt_coord[::-1])
            #cv2.circle(img, alt_coord, radius=4, color=[100,150,230], thickness=-1)
        
        # warning-specific changes
        if display_command < 3:
            img[warning_sign_pos[0]:warning_sign_pos[1],
                warning_sign_pos[2]:warning_sign_pos[3]] = np.where(wsign_included,
                        wsign, img[warning_sign_pos[0]:warning_sign_pos[1],
                                   warning_sign_pos[2]:warning_sign_pos[3]])
        # update display
        #self.vid.writeFrame(img[:,:,::-1])
        img = cv2.resize(img, (pix_size[0]*2, pix_size[1]*2)) ### new for bigger display
        cv2.imshow('Merging Assist', img)
        cv2.waitKey(1)
        self.audio.changeDisplay(display_command)
        
    def stopDisplay(self):
        self.audio.stopDisplay()
        cv2.destroyWindow('Merging Assist')
        #self.vid.close()
        
    
    
if __name__ == '__main__':
    import time
    #merging_car = np.array((622335.2, 3361937.5)) redriver
    merging_car = road.road2Global(206., 2)  # balcones
    alty = (100, 1, 10, 0)
    k = Display(merging_car)
    k.changeDisplay(3, merging_car, [alty])
    time.sleep(10.)
    k.changeDisplay(4, merging_car, [])
