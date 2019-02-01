# CARSTOP project prototype code
This code performs five different collision warning applications, using a suite of sensors and devices common to current and/or self-driving vehicles.

## Requirements
### Hardware
The applications are not designed to rely on one particular device, instead using the code from the sensors folder to obtain useful information. However, implicitly the code assumes device performance will be similar to the devices used in the project.

1. Camera - Logitech C920, positioned on front dashboard
2. Lidar - Quanergy M8 and Velodyne VLP-16, on top of vehicle
3. Radar - Delphi ESR, on front license plate
4. Communication and Positioning - Cohda DSRC OBU, antenna on top of vehicle

### Software
All applications were written in Python 2.7. Each sensor may require packages or tools as stated in their readme.  
Python packages needed by most codes are:
numpy and scipy (already present with most distribution downloads)
opencv3 https://anaconda.org/menpo/opencv3
pyaudio https://anaconda.org/anaconda/pyaudio - may require additional steps depending on OS, see https://people.csail.mit.edu/hubert/pyaudio/

Packages needed for specific apps or sensors are:
utm https://anaconda.org/conda-forge/utm
cantools https://anaconda.org/conda-forge/cantools

### GPU
CUDA, and a device with a compatible GPU, is not technically required for any apps to function.
But the pedestrian and workzone apps are too slow to be practical without it.
Installation of CUDA is too device-dependent to be explained here...

## Usage
Files should be run from this folder using Python's module-based command. For instance:  
"python -m prototype.sensors.quanergyLidarPoint"  
instead of  
"python prototype/sensors/quanergyLidarPoint.py"  
Why the "-m"? Because this code was set up as a package, but we don't want to specify a way to install the package. If this folder is added as a package using pip, or by directly adding it to the Python PATH, then the normal python command style will work.

The sensors folder contains codes that extract data for real-time use from each of the sensors used in this project. Each sensor file can be run on its own to see a visualization of its output. For instance: 
python -m prototype.sensors.quanergyLidarPoint 
shows a scatter-plot video of the lidar's detections.

Each app folder has a file called app.py that runs it, such as:
python -m prototype.app_overtaking.app
Each folder has a file called options.py that stores options such as output or sensor settings.
All applications have an option called real_vs_simulated.
If real_vs_simulated=True, the app runs in real-time using the appropriate sensors.
If False, the app uses recorded data or a simulation - the data or simulation will also be visualized.
The rest of the files in an app folder are app-specific and usually don't need to be touched unless modifying the app.

