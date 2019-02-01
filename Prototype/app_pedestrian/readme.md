# Pedestrian Warning Application
This application detects pedestrians and sends a warning if one is close, or predicted to come close, to the front of the vehicle. The simulated version of the application uses pre-recorded sensor data (not included in repository)

## Requirements
1. A GPU and CUDA installed.
2. darknet  
   A: Download from [github.com/pjreddie/darknet](https://github.com/pjreddie/darknet)  
   B: In the makefile, change GPU=0 to GPU=1  
       you may also need to set CPP=gcc  
   C: open a terminal in this folder and run `make`  
   D: Download the YOLOv2 weights from [pjreddie.com/darknet/yolo](https://pjreddie.com/darknet/yolo/), put them in the darknet folder
3. change "options.py" to have the correct location for your darknet folder
4. Python with numpy, opencv, and pyaudio

Required sensors for real version: Quanergy lidar, camera

Calibration matrices are needed to match camera and lidar data - we used a graphical tool available at [https://github.com/twankim/svdped/blob/master/calib_gui.py](github.com/twankim/svdped/blob/master/calib_gui.py)