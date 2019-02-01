## Workzone Application
To run this demo, you need:

1. A GPU and CUDA installed.
2. darknet  
   A  Download from github.com/pjreddie/darknet  
   B  In the makefile, change GPU=0 to GPU=1; you may also need to set CPP=gcc  
   C  open a terminal in this folder and run "make"  
   D  Download the YOLOv2 weights from pjreddie.com/darknet/yolo, put them in the darknet folder  
3. change darknet_files.py to have the correct location for your darknet folder  
   A  also change coco.data to point to your darknet folder's location  
4. Python with numpy and opencv


This demo does not have a test version, sorry! You need a camera connected to run.

This demo actually requires two codes being run at once on the server side. "capture.py" uses the camera and saves information to the server. "ios_server.py" connects to clients and sends them that information.
They can each be run like the other apps, for instance `python -m prototype.app_workzone.capture`

The capture code will display the video and print detected objects for reference.
