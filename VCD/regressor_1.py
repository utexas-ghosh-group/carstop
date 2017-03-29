# -*- coding: utf-8 -*-
"""
Specifies the regression model for the left turn scenario with
one second of prediction, and simulates data with which to train this regressor.

The random scenario generation from the simulations could be used to train
regression models. However, models that are trained on these admittedly
specific examples might perform well on these examples but not other scenarios,
resulting in a somewhat dishonest evaluation. To avoid this, the vehicles'
initial positions are generated from a uniform distribution in this code, rather
than the moving-backwards generation of the simulate_1 code. Additionally, the
variance in the initial position (stored here as 'noise') is not the same as in
the simulate_1 code.

For this model, input features to the regressor are each vehicle's position and
velocity, totalling four features.

For the first two experiments, uncertainty in the vehicles' initial
positions is actually not included in the model. This lowers the dimensionality
and makes training much more feasible, but will cause the alarm to be highly
inaccurate if the vehicle's starting positions are poorly known (then again, in
this case the alarm will probably suck anyway).

These features are transformed so they are bounded between -1 and 1 before
being used by the regressor.
"""

import alarms, motionModels
import numpy as np
from sklearn.model_selection import train_test_split
from scoring import AUC,ROC
from sklearn.neural_network import MLPRegressor
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.tree import DecisionTreeRegressor
import time

model_name = 'sim_1'

mean_means = np.array((-5, 10))
mean_bounds = np.array((25, 20))
noise = np.diag([1e-6, 1e-6]) # essentially zero noise
truthMM1 = motionModels.MM_LineCV('left-right', noise)
truthMM2 = motionModels.MM_LineCV('right-down', noise)
timeres = .1
timelen = 1.
times = np.zeros((int(timelen/timeres),)) + timeres


def createTruth(npoints, nrepeats=4000):
    """
    Generates a range of initial states for both vehicles, then simulates from
    each one a certain number of times.
    """
    ## generating a grid of features
    res = int(npoints**.25)
    npoints = res**4
    x_0 = np.linspace(-1.,1.,res)
    X_0 = np.empty((npoints, 4))
    for feature in range(4):
        X_0[:,feature] = np.repeat(np.tile(x_0, res**feature), res**(3-feature))    
    ## randomly generating features
    #X_0 = np.random.uniform(-1,1,size=(npoints,4))
    
    y_0 = []
    for point in range(npoints):
        x = X_0[point,:]
        mean1 = x[:2] * mean_bounds + mean_means
        mean1 = motionModels.initialState_normal(mean1, noise*.1)
        mean2 = x[2:4] * mean_bounds + mean_means
        mean2 = motionModels.initialState_normal(mean2, noise*.1)
        out = alarms.alarm_MCS(mean1, mean2, truthMM1, truthMM2, times,
                                    nrepeats)
        y_0 += [out[0]]
    y_0 = np.array(y_0)
    
    np.save(model_name+'_X.npy', X_0)
    np.save(model_name+'_Y.npy',y_0)
    
        
def scoreModel(truth, pred, logit):
    """ Gathers R-squared (regression quality measure) and area-under-ROC
    (classification quality measure); optimal value for both is 1"""
    if logit:
        pred = 1./(1+np.exp(-pred))
    else:
        pred = np.minimum(np.maximum(pred,0.),1.)
    print( "test R^2 {:.3f}".format(
        1-np.sum((pred - truth)**2.)/np.sum((np.mean(truth)-truth)**2.) ))
    print( "AUC {:.3f}".format( AUC(*ROC(truth, pred)) ))
     
    
def trainModels():
    """ Loads the data created by createTruth() and tries several regressors
    scores each and saves models (currently only saving MLP) """
    X_0 = np.load(model_name+'_X.npy')
    y_0 = np.load(model_name+'_Y.npy')
    
    ## Transforms the probability values using the logit function, which is
    ## often used in classification models. Did not have a major effect.
    logit = False
    
    print( "y mean = {:f} , RMSR = {:f}".format(np.mean(y_0), np.std(y_0) ))
    
    X_train, X_test, y_train, y_test = train_test_split(X_0,y_0,test_size=.01)
    if logit:
        y_train[y_train > .9999] = .9999
        y_train[y_train < .0001] = .0001
        y_train = np.log(y_train / (1 - y_train))
    
    ## multi-layer perceptron
    print("MLP")
    mlp = MLPRegressor(hidden_layer_sizes=(150,), tol=1e-10, max_iter=500)
    mlp.fit(X_train, y_train)
    print( "train R^2 {:f}".format( mlp.score(X_train, y_train) ))
    scoreModel(y_test, mlp.predict(X_test), logit)
    ff = time.time()
    mlp.predict(X_0[:100])
    print( "runtime for 100 samples: {:.1e}".format( time.time()-ff ))

    ## gradient boosting, a slow but highly regarded ensemble regressor
    print("GBR")
    gbr = GradientBoostingRegressor(max_depth=4)# default 3
    gbr.fit(X_train,y_train)
    print( "train R^2 {:f}".format( gbr.score(X_train, y_train) ))
    scoreModel(y_test, gbr.predict(X_test), logit)
    ff = time.time()
    gbr.predict(X_0[:100])
    print( "runtime for 100 samples: {:.1e}".format( time.time()-ff ))

    ## regression trees, very fast in training and use, but easily overfit data
    print("DT")
    dt = DecisionTreeRegressor(max_depth=14)
    dt.fit(X_train,y_train)
    print( "train R^2 {:f}".format( dt.score(X_train, y_train) ))
    scoreModel(y_test, dt.predict(X_test), logit)
    ff = time.time()
    dt.predict(X_0[:100])
    print( "runtime for 100 samples: {:.1e}".format( time.time()-ff ))
    
    ## only saved MLP
    np.save(model_name+'_MLP.npy',mlp)
    
    
class Model():
    """
    The formatting of features was difficult to generalize across examples, so
    each example has an object that formats a scenario and applies the model.
    """
    
    def __init__(self, name='MLP'):
        self.model=np.load(model_name+'_'+name+'.npy').item()
        
    def alarm(self, vehicle1, vehicle2, MM1, MM2, times):
        """input and output match that of the functions in alarms.py"""
        starttime = time.time()
        x = np.array(([0.]*4,))
        x[0,:2] = (vehicle1.mean[:2] - mean_means) / mean_bounds
        x[0,2:4] = (vehicle2.mean[:2] - mean_means) / mean_bounds
        pred = self.model.predict(x)[0]
        return (min(max(pred,0.),1.) , time.time() - starttime)
      
        
""" uncomment createTruth or trainModels() to easily run from the command line
    comment it again before running the main simulator! """
if __name__=='__main__':
    #pass
    #createTruth(10**6)
    trainModels()