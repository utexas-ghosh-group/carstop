The raw data files (the trajectory .txts and preppedData.csv) are only on Dropbox, as they exceed the file size limit of Github.

These files use the preppedData.csv file, which is the Lankershim data with some transformations and unnecessary features cut.  Details are in prepDetails.txt.

DAQSection.m : gathers the data around one of the NGSIM sections (i.e. section 3, which Rahi was using) in local coordinates.  I used this to decide where to put segment lines.

DAQSegment.m : once you set up a segment line (defined by pointA and pointB), this code will extract all the appropriate trajectories and save them to a "segmentXX.mat" file.  If you run it multiple times, change the name of the saved file each time.

plotter.m : the first half plots data from DAQSection.m and lets you mess with the segment lines.  The second plots data from DAQSegment.m, so you can see what the result looks like.
