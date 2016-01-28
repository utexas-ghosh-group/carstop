# -*- coding: utf-8 -*-
"""
Include prediction classes/functions in here!
last modified 11/10/15

Predictors are classes, with __init__(trueTrajectory, other args)
and predict(vData, predictTimes)
    trueTrajectory = pd dataframe with the vehicle's actual states over time
        most predictors won't use this, but to make things flexible every
        predictor should include the argument
    VData = pd dataframe with vehicle's sensed states up to current time
"""
#from rearEndPredict import RearEndPredict
#from rearEndConstant import RearEndConstant
from rearEndCA import RearEndCA
from rearEndKalman import RearEndKalman
from genericNoisePredict import GenericNoisePredict
#from kalmanPredict import KalmanPredict_CV
#from kalmanPredict_CA_angle import KalmanPredict_CA_angle
from kalmanPredict_line import KalmanPredict_line
from CV_line import CV_line
#from gpmlPredict import GpmlPredict
