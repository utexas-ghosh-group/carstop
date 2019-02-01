real_vs_simulator = True # no simulated version implemented

# this file stores the current knowledge about the zone
# it is written to by the capture software, then sent to client devices
mapfile = 'prototype/app_workzone/mapfile.txt'

# port for communication
TCP_PORT = 8000

## files and settings for darknet
darknet_location = "/home/motrom/Downloads/kitti_devkit/darknet-master/"
#darknet_location = "/home/michael_motro/darknet2018/"
code = darknet_location+"libdarknet.so"
data = "prototype/app_workzone/coco.data"
weights = darknet_location+"yolov2.weights"
cfg = darknet_location+"cfg/yolov2.cfg"

## calibration
focalLength = 1309.859 # camera's focal length, assume roughly equal in both directions
knownHeight = 71 * .0254 # height of camera in meters