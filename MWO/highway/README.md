This is code for experiments pertaining to the paper "Measurement-wise Occlusion in Multi-object Tracking". Specifically, this code contains a simulated four-lane highway (or a cheap copy of Frogger, if you prefer), as well as code to generate measurements following either of the occlusion formulations covered in the paper. There are also 3 multi-object trackers with different ways of handling occlusion. Note that this experiment and its results are only discussed in the arXiv version of the paper.

# setup
Python 2 was used, Python 3 will probably function.

Packages used:

* numpy
* scipy
* scikit-video if you want to use video

# usage
## frogger
The code for simulating. If run on its own it will generate a short example video. Note that the types of occlusion can be altered by switching between sense_OWO() and sense_MWO().

There are a number of parameters that alter the simulation, such as:

* length of the highway section
* number of lanes
* lane width
* minimum and maximum vehicle length
* minimum and maximum vehicle speed
* probability of detection (not accounting for occlusion)
* false positive generation rate
* magnitude of white noise applied to each measurement

## froggerMWO
A track-oriented multi-bernoulli tracker implemented via particle filter. Each vehicle has constant speed, and each vehicle's lane is assumed to be known reliably. So filtering is fairly easy!

Measurement-wise occlusion probability is calculated exactly as the probability that a measurement from this object lies strictly within the occluded zone of the nearer lanes, using cdf().

Object entry (birth) is handled by adding objects (bernoulli components) every timestep. The existence probability of these objects can be altered. This and the number of particles are the only major hyperparameters.

Scored using GOSPA(distance = sqrt(distance_position^2 + distance_length^2), c = 5 meters, p = 1).

## froggerOWOev
Exactly like froggerMWO except for the occlusion handling. Occlusion probability for one object is obtained by

* Taking expected value of every object in a nearer lane
* For each nearer object, calculate how many meters it covers the left edge and right edge of this object (positive meaning it covers by a certain number of meters, negative meaning it is that many meters from covering this object).
* The 'probability' of individual occlusion for this object is given as (1 - e^(3 left_cover))*(1 - e^(3 right_cover)) (nearer object's existence probability)
* Combine to get the joint probability of no nearer objects occluding this object.

The 3 is obviously a guessed parameter, I have not really played around with different individual occlusion terms.

## froggerOWOgrid
Sets up a grid on the angles of the sensor. The probability of full occlusion for a particle is equal to the lowest probability of occlusion for a cell that the object overlaps with. The cells are updated for each level by summing the existence weight of each particle that overlaps with the cell. Currently 101 cells are used, higher resolutions have been tried and give marginal improvement.