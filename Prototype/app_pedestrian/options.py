real_vs_simulator = False

## options for real app
save = False # if True, save images
WITHOUT_LIDAR = False # if False, save lidar rotations when saving images
saveto = 'prototype/app_pedestrian/d' # will save images/points here

## options for recorded-data app
image_folder = 'prototype/app_pedestrian/example_data/image/'
lidar_folder = 'prototype/app_pedestrian/example_data/lidar/'

## files and settings for darknet
darknet_location = "/home/motrom/Downloads/kitti_devkit/darknet-master/"
code = darknet_location+"libdarknet.so"
data = "prototype/app_pedestrian/coco.data"
weights = darknet_location+"yolov2.weights"
cfg = darknet_location+"cfg/yolov2.cfg"

## other parameters
calib_intrinsic_file = 'prototype/app_pedestrian/calib_intrinsic.txt'
calib_extrinsic_file = 'prototype/app_pedestrian/calib_extrinsic.txt'
HIDE_BOXES = False # if True, doesn't add pedestrian boxes to video (just audio warning)
DETAIL = False # if True, print more information