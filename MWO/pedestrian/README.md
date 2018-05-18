This is the code for experiments pertaining to the paper "Measurement-wise Occlusion in Multi-object Tracking", presented at the 2018 IEEE Information Fusion Conference. Specifically, this code uses a track-oriented multi-bernoulli filter to track pedestrians on videos from the MOT17 benchmark. Videos 4 and 5 are the ones it was tailored for, and only results on video 4 are reported. We're not using the usual MOT scoring system so we need the ground truth files, and video 5 doesn't have ground truth.

# setup
The FRCNN-04 folder from the MOT17 challenge is the only data that is needed.

Python 2 was used, Python 3 will probably function.

Packages used:

* numpy
* scipy
* motmetrics (https://github.com/cheind/py-motmetrics)
* scikit-video if you want to use video

# usage
## prepareDataset
Organizes the benchmark's det.txt file, which is for some reason not time-ordered. It also removes non-human detections from the ground truth file.

## track
The main script to run. Usage options such as file locations, whether to visualize, and the type of occlusion handling are hard-coded at the beginning of the file.

## trackmodel
This file contains hyperparameter selection and functions for parallel prediction, likelihood calculation, and updating of all objects/components. It was designed so that the main scripts could focus on the multi-object tracking without specifying the underlying single-object models. This split is not currently complete - a couple things like pruning still rely the format of object states.

Of course, a variety of details were left out of the paper due to lack of space and presumed reader interest. For one, we are actually running trackers at half the FPS of the standard MOT benchmark. That is, we're updating the trackers on every other image frame. This can be changed by modifying the "skip" parameter. Reporting and scoring still occurs at the original 30fps. We deemed the lower framerate more realistic, given that a deep-learning algorithm is generating these bounding boxes.

In the same way that occludability was handled like a simple interactive multiple model, the probability of detection was added as an object feature with a transition probability. This is a more fair comparison than a completely context-less tracker, and is a good idea in general for high-framerate tracking where failed detections usually occur in a row. The settings for the baseline occlusion-less tracker were:

~~~
detectability_stationary = .9
detectability_half_second_ratio = .99
~~~

The settings for both occlusion trackers were:

~~~
detectability_stationary = .93
detectability_half_second_ratio = .99
occludability_stationary = .8
occludability_half_second_ratio = .8
~~~

You can use a vanilla tracker w.r.t. the probability of detection by switching the detectability_half_second_ratio to 1, but it does in fact perform worse.

Occlusion was determined as the product of independent occlusion probabilities from each (potentially) occluding object.
The occlusion probability between two objects is
fix(2 * area_intersection / area_occluded) * fix((occluding_bottom - occluded_bottom)/occluding_height)
where fix() caps low values at 0 and high values at 1.

The tracking method is deterministic, so results from running this code should be very similar to those in the paper.

## score
Uses a ground truth file from prepDataset.py and a detection file saved by track.py. Reports MOT benchmark metrics (with some changes such as precision definition, see motmetrics website) and GOSPA.