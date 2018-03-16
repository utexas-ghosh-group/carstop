This is the code for experimental results in a submission to the 2018 Information Fusion conference, titled "Measurement-wise Occlusion in Multi-object Tracking". If the paper is published, the code will be cleaned and documented much more than it currently is. At the moment, it should only be of interest to very fastidious reviewers...

# setup
The FRCNN-04 folder from the MOT17 challenge is the only data that is needed.

Packages used:

* numpy
* scipy
* motmetrics (https://github.com/cheind/py-motmetrics)
* scikit-video if you want to use video

# usage
## prepareDataset
Organizes the benchmark's det.txt file, which is for some reason not time-ordered. It also removes non-human detections from the ground truth file.

## track
The main function. All usage options, such as file locations, whether to visualize, and algorithm hyperparameters, are hard-coded at the beginning of the file.

Of course, a variety of details were left out of the paper due to lack of space and presumed reader interest. For one, we are actually running trackers at half the FPS of the standard MOT benchmark. That is, we're updating the trackers on every other image frame. This can be changed by modifying the "skip" parameter. Reporting and scoring still occurs at the original 30fps. We deemed the lower framerate more realistic, given that a deep-learning algorithm is generating these bounding boxes.

In the same way that occludability was handled like a simple interactive multiple model, the probability of detection was added as an object feature with a transition probability. This is a more fair comparison than a completely context-less tracker, and is a good idea in general for high-framerate tracking where failed detections usually occur in a row. The parameters for the baseline occlusion-less tracker were:

~~~
detectability_stationary = .9
detectability_half_second_ratio = .99
occludability_stationary = 0.
occludability_half_second_ratio = 1.
~~~

The parameters for the occlusion tracker were:

~~~
detectability_stationary = .93
detectability_half_second_ratio = .99
occludability_stationary = .8
occludability_half_second_ratio = .8
~~~

You can use a vanilla tracker w.r.t. the probability of detection by switching the detectability_half_second_ratio to 1, but it does in fact perform worse.

## score
Uses a ground truth file from prepDataset.py and a detection file saved by track.py.