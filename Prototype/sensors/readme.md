# Sensor code
## Usage
Each code extends Python's multiprocessing.Process() class, as well as implementing `__enter__()` and `__exit__()` for Python's `with` statements. The `__enter__()` function starts the separate process, and the `__exit__()` function terminates it. Hence the syntax for using sensors is simply:

```python
with Sensor1(input_options) as sensor1, Sensor2(input_options) as sensor2:
    # do stuff with sensor1 and sensor2
print("the sensors get shut down before this statement prints")
```

The sensors do not have a standardized output type or function. Most use a `multiprocessing.Queue()` to send arrays or tuples of data. The functions and outputs are explained in comments at the top of each sensor file. All sensors have a `__main__` section that graphically displays the sensor's output. This demonstrates the sensor's usage and can be used as a quick test of functionality, by running  
```python -m prototype.sensors.Sensor1.py```

Most important sensor parameters are hardcoded in each file. However, the digital output for some of the sensors is proprietary and hence left out of this repository. Namely, the Delphi ESR's dbc file and the Quanergy lidar's input format are not present.

Sensor processing choices are also hardcoded in each file and might be changed by users. For instance, lidar can specify which lasers to use.

Helper files are:  
1. point2rect.py - transforming a series of points into line segment estimates, used in lidar preprocessing  
2. connectors.py - socket connections with timeout checks and non-blocking reads, used for V2V and IMU connections

## Dependencies

Radar  
- the aforementioned dbc file  
- The Kvaser's driver must be installed https://www.kvaser.com/downloads-kvaser/. This can occasionally can fail, and frankly I have no idea how drivers work. If it is working at first then stops, you can uninstall and re-install it.  
- cantools package of Python

Lidar  
- The ethernet connection with the LIDAR should be set to an IP that the LIDAR is expecting (google "static IP" and your OS to learn how). For instance, for Velodyne the IP should be 192.168.1.something

Camera  
- ffmpeg  
- for opencvCamera, OpenCV3 (for Python)

V2V  
- the utm Python package