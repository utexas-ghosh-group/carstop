import numpy as np
import cv2
import time
import contextlib
import os

import prototype.app_pedestrian.darknet as darknet
#from prototype.sensors.velodyneLidarRobust import LIDAR
from prototype.sensors.quanergyLidarRobust import LIDAR
from prototype.app_pedestrian.pedestrianAudio import AudioWarning
#class AudioWarning():
#    def __init__(self): pass
#    def __enter__(self): return self
#    def __exit__(self,a,b,c): pass
#    def change(self, bs): pass

import prototype.app_pedestrian.options as args
REAL = args.real_vs_simulator

intrinsic = np.loadtxt(args.calib_intrinsic_file)
pose = np.loadtxt(args.calib_extrinsic_file)
if args.real_vs_simulator:
    pose[:,:2] *= -1 # this is for Quanergy rather than Velodyne

@contextlib.contextmanager
def dummy_context_mgr():
    yield None

def get_real_coord(pedestrian):
    for ped in pedestrian:
        x,y = ped['center'][1],ped['center'][0]
        u,v,w = (x-intrinsic[1,2])/intrinsic[1,1],(y-intrinsic[0,2])/intrinsic[0,0],1
        magnitude = ped['dist']/np.hypot(u,w)
        u *= magnitude
        v *= magnitude
        w *= magnitude
        ped['xyz'] = [u,v,w]
def predictMovement(pedestrian):
    for ped in pedestrian:
        ped['xyz'] = np.add(ped['xyz'],ped['displacement'])
    return None

def track_pedestrian(detected, known, ID):
    get_real_coord(detected)
    if(known.size == 0):
        for ped in detected:
            ped['id'] = ID
            ped['displacement'] = [0,0,0]
            ped['lifetime'] = 5
            ID += 1
        return detected, ID
    predictMovement(known)
    center = np.array([i['xyz'] for i in known])
    result = []
    missing = np.ones(len(known), dtype=bool)
    for ped in detected:
        displacement = np.array(ped['xyz'] - center)
        dist = np.hypot(displacement[:,0],displacement[:,2])
        match = dist<0.2    
        ind = np.argwhere(match == True).reshape(-1)
        nmatch = ind.size
        TF = False
        if(nmatch == 1): # "known object is detected.
            ped['displacement'] = displacement[ind[0]]+known[ind[0]]['displacement']
            ped['id'] = known[ind[0]]['id']
            ped['lifetime'] = 5
            result.append(ped)
            TF = True
            missing[ind[0]] = False
        elif(nmatch > 1):
            closest = np.min(dist[match])
            ind = np.argwhere(dist==closest).reshape(-1)
            ped['displacement'] = dist[ind[0]]+known[ind[0]]['displacement']
            ped['id'] = known[ind[0]]['id']
            ped['lifetime'] = 5
            result.append(ped)
            TF = True
            missing[ind[0]] = False
        else: # newly detected object
            ped['displacement'] = [0,0,0]
            ped['id'] = ID
            ped['lifetime'] = 5
            ID += 1
            result.append(ped)
    missing = known[missing]
    #print(missing)
    for ped in missing:
        break
        ped['lifetime'] -= 1
        diff = ped['center'] - [ped['topleft']['y'],ped['topleft']['x']]
        u,v = ped['xyz'][0:2]/ped['xyz'][2]
        x,y = u*intrinsic[1,1]+intrinsic[1,2],v*intrinsic[0,0]+intrinsic[0,2]
        ped['center'] = np.array([y,x]).astype(int)
        ped['topleft']['y'] = ped['center'][0] - diff[0]
        ped['topleft']['x'] = ped['center'][1] - diff[1]
        ped['bottomright']['y'] = ped['center'][0] + diff[0]
        ped['bottomright']['x'] = ped['center'][1] + diff[1]
        if ped['lifetime'] != 0:
            result.append(ped)    
    result = np.array(result)
    return result, ID

def lidar_dist(topleft, bottomright, points):#lidar):
    # transform lidar points to image reference
    #points = lidar.dot(pose[:3,:3].T)+pose[:3,3]
    dist = points[:,2]
    points = points[:,:2]/points[:,2:3]
    
    # find location at image
    points *= [intrinsic[0,0],intrinsic[1,1]]
    points += [intrinsic[0,2],intrinsic[1,2]]
    points = points.astype(int)
    # use only valid points
    valid = np.all(points >=1, axis=1)
    valid &= np.all(points+1 < [bottomright['y'],bottomright['x']],axis=1)
    valid &= np.all(points+1 > [topleft['y'],topleft['x']],axis=1)
    valid &= dist > 1.
    valid &= dist < 50
    
    dist = dist[valid]
    #print(dist.size)
    if dist.shape[0] == 0 :
        dist = 9999
    else:
        dist = np.min(dist)
    return dist


def main():
    if not os.path.exists('{}_lidar'.format(args.saveto)):
        os.makedirs('{}_lidar'.format(args.saveto))
    if not os.path.exists('{}_image'.format(args.saveto)):
        os.makedirs('{}_image'.format(args.saveto))
    
    # Prepare camera/lidar connection
    if REAL:
        cam = cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT,720)
        ret, img = cam.read()
        if not ret:
           cam.release()
           raise Exception
    # Prepare yolo
    yolo_net, yolo_meta, yolo_im = darknet.load(720, 1280)
    
    alarm = False
    close = False
    near = False
    
    start = time.time()
    try:
        with AudioWarning() as warning, LIDAR() if REAL else dummy_context_mgr() as lidar:
            unique_ped = np.array([])
            ped_ID = 0
            index = 24
            while(True):
                index +=1
                sub=time.time() if args.DETAIL else None
                """Read image from camera/path"""
                if REAL:
                    ret, img = cam.read()
                    assert ret
                else:
                    img = cv2.imread(args.image_folder+'{:06d}.png'.format(index))
                assert img.shape==(720,1280,3)
                assert img.dtype==np.uint8
                if args.DETAIL:
                    nexttime = time.time()
                    print('Camera read : {}s'.format(nexttime-sub))
                    sub = nexttime
                """Read lidar from lidar/.npy file"""
                if REAL:
                    lidar_xyz = lidar.get(timeout=0.1)
                    lidar_xyz = np.array(lidar_xyz)
                else:
                    lidar_xyz = np.load(args.lidar_folder+'{:06d}.npy'.format(index))
#                    lidar_xyz = np.fromfile(args.lidar_folder+'{:06d}.bin'.format(index))
#                    assert len(lidar_xyz)%3 == 0
#                    lidar_xyz = lidar_xyz.reshape((len(lidar_xyz)//3, 3))
                lidar_xyz = lidar_xyz.dot(pose[:3,:3].T)+pose[:3,3]
                if args.DETAIL:
                    nexttime = time.time()
                    print('Lidar read : {}s'.format(nexttime-sub))
                    sub = nexttime
                """Detect pedestrian by pre trained YOLO v2 network (Darkflow)"""
                result = darknet.detect(yolo_net, yolo_meta, yolo_im, img[:,:,::-1]) # rgb2bgr
                if args.DETAIL:
                    nexttime = time.time()
                    print('YOLO process : {}s'.format(nexttime-sub))
                    sub = nexttime
                """Detecting only object labeled as person"""
                human = []
                for obj in result:
                    if obj['label'] == 'person':
                        obj['center'] = np.array([(obj['topleft']['y']+obj['bottomright']['y'])/2,(obj['topleft']['x']+obj['bottomright']['x'])/2]) # center represent centroid of detection boxes in [y,x] format.
                        obj['dist'] = lidar_dist(obj['topleft'], obj['bottomright'], lidar_xyz)
                        human.append(obj)
                """Eliminate duplicated boxes"""
                unique = []
                for i, ped in enumerate(human):
                    ox1,oy1 = ped["topleft"]["x"],ped["topleft"]["y"]
                    ox2,oy2 = ped["bottomright"]["x"],ped["bottomright"]["y"]
                    marker = True
                    for j, obj in enumerate(human):
                        if(i<j):
                            x1,y1 = obj["topleft"]["x"]-10,obj["topleft"]["y"]-10
                            x2,y2 = obj["bottomright"]["x"]+10,obj["bottomright"]["y"]+10
                            if ox1>x1 and ox2<x2 and oy1>y1 and oy2<y2:
                                marker = False
                    if marker == True:
                        unique.append(i)
                human = np.array(human)
                human = human[unique]
                """Tracking pedestrian"""
                unique_ped, ped_ID = track_pedestrian(human, unique_ped, ped_ID)
                """Draw boxes on images and print by openCV2"""
                if not args.HIDE_BOXES:
                     for ped in unique_ped:
                        color = (0,0,0)#white
                        topleft = (int(ped['topleft']['x']),
                                   int(ped['topleft']['y']))
                        bottomright = (int(ped['bottomright']['x']),
                                       int(ped['bottomright']['y']))
                        cv2.rectangle(img, topleft,bottomright,(0,0,255),2)
                        #cv2.putText(img, 'ID : {}'.format(ped['id']), (topleft[0],topleft[1]-30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color)
                        if ped['dist'] < 100:
                            cv2.putText(img, '{:.1f} m'.format(ped['dist']), (topleft[0],topleft[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color,2)
                            ped_future = ped['xyz'] + np.array(ped['displacement'])*10
                            ped_curr_lon = ped['xyz'][2]
                            ped_curr_lat = abs(ped['xyz'][0])
                            ped_future_lon = ped_future[2]
                            ped_future_lat = abs(ped_future[0])
                            ped_curr_close = ped_curr_lon < 4 and ped_curr_lat < 1.5
                            ped_curr_near = ped_curr_lon < 20 and ped_curr_lat < 4
                            ped_future_close = ped_future_lon < 5 and ped_future_lat < 2.5
                            ped_future_near = ped_future_lon < 15 and ped_future_lat < 4
                            if ped_curr_close or ped_future_close:
                                close = True
                            elif ped_curr_near or ped_future_near:
                                near = True
                cv2.imshow('yolo + lidar', img)
                """Make a warning sound"""
                
                if index%5==0:
                    if close == True:
                        alarm = True
                        print("CLOSE")
                    elif near == True:
                        alarm = False if alarm else True
                        print("Near")
                    else:
                        alarm = False
                        print("Safe")
                    warning.change(alarm)
                    close, near = False, False
                cv2.waitKey(5)
                if args.DETAIL:
                    nexttime = time.time()
                    print('drawing : {}s'.format(nexttime-sub))
                    sub = nexttime
                print('Whole process working @ {:3.1f} fps'.format(1./(time.time()-start)))
                start = time.time()
                if args.save==True:
                    cv2.imwrite('{}_image/{:06d}.jpg'.format(args.saveto,index), img)
                    if not args.WITHOUT_LIDAR:
                        np.save('{}_lidar/{:06d}'.format(args.saveto,index),lidar_xyz)
                    if args.DETAIL:
                        nexttime = time.time()
                        print('saving : {}s'.format(nexttime-sub))
    except KeyboardInterrupt:
        pass
    print('\nThis process is successfully done')
    cam.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()