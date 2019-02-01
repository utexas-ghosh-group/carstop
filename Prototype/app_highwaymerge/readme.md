# Highway Merge Application

Receives V2V from oncoming vehicles, and alerts the driver whether merging onto the highway (from a standstill) is safe.

Running `prototype.app_highwaymerge.app` will show a live map of the road with oncoming vehicles.
Running `prototype.app_highwaymerge.app_basicdisplay` will instead show static safe or warning images.
Both use audio warnings when vehicles are oncoming.

## Requirements
Requires: Python with numpy, opencv, and pyaudio packages.

Real version requires Cohda unit connected.

This code requires a short map (several hundred meters) of the road the vehicle will merge onto.
Several example maps on the UT campus have been made and are in the "roads" folder:  
1. Innovation Road in UT East Pickle campus  
2. Balcones Drive in UT West Pickle campus  
3. Red River Road in UT main campus

New maps can be made according to the format described in "roads.py"

