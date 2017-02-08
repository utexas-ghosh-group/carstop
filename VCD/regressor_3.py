# -*- coding: utf-8 -*-
"""
Training data creation and regression for the bicycle-model example. Most
documentation is in the first regressor module.

Unlike the first two examples, the uncertainty in each vehicle's initial
position is actually included in the training. This results in a much higher
feature count, to the point where the models were never sufficiently trained...
however, this is the more correct way to use the model. State-of-the-art vehicle
motion models will likely have many variables or knobs and thus have a similarly
high feature count.
"""

import alarms, motionModels
import numpy as np
import time
from sklearn.model_selection import train_test_split
from sklearn.neural_network import MLPRegressor
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.tree import DecisionTreeRegressor
from scoring import AUC,ROC

model_name = 'sim_3'

timeres = .1
timelen = 1.
times = np.zeros((int(timelen/timeres),)) + timeres
mean_bounds = np.array((20, 20, np.pi, 30, 3, np.pi/2))
stdev_bounds = np.diag((10, 10, np.pi / 3**.5, 5, 1, np.pi/4)) / 4.


def randomCovarianceMatrix(ndim):
    """
    It is hard to randomly sample legitimate covariance matrices.  Here SVD is
    applied to a random matrix, and the resulting vectors are used to construct
    a symmetric, positive semi-definite, and correctly bounded matrix.
    """
    a = np.random.uniform(-1., 1., size=(ndim,ndim))
    u,s,v = np.linalg.svd(a)
    s2 = np.random.uniform(0.,1.,size=ndim)
    newmat = u.dot(np.diag(s2)).dot(u.T)
    return newmat
    
    
def createTruth(npoints, nrepeats=4000):
    """
    It can take a very long time to generate this data - 1 million points will
    take a day on a good desktop.
    
    We keep the first vehicle at coordinate (0, 0) to lower the dimensionality.
    Hypothetically you can also adjust the angles so that the first vehicle is
    always facing one direction, but the distribution is no longer normal
    and the covariances are complicated to transform.
    """

    means1 = np.random.uniform(-1., 1., size=(npoints, 4))
    covmats1 = np.empty((npoints,6,6))
    for point in range(npoints):
        covmats1[point,:,:] = randomCovarianceMatrix(6)
    means2 = np.random.uniform(-1., 1., size=(npoints, 6))
    covmats2 = np.empty((npoints,6,6))
    for point in range(npoints):
        covmats2[point,:,:] = randomCovarianceMatrix(6)
    ## get upper part of covariance matrices
    X_0 = np.concatenate((means1, covmats1[:,0,:], covmats1[:,1,1:],
                          covmats1[:,2,2:], covmats1[:,3,3:], covmats1[:,4,4:],
                          covmats1[:,5,5:], means2, covmats2[:,0,:],
                          covmats2[:,1,1:], covmats2[:,2,2:], covmats2[:,3,3:],
                          covmats2[:,4,4:], covmats2[:,5,5:]), axis=1)
    
    y_0 = []
    noise = np.diag([1., 1., .5, .1, 0.05, 0.01])
    MM1 = motionModels.MM_Bicycle(noise)
    MM2 = motionModels.MM_Bicycle(noise)
    for point in range(npoints):
        mean1 = np.append([0,0], means1[point,:] * mean_bounds[2:])
        cov1 = stdev_bounds.dot(covmats1[point,:,:]).dot(stdev_bounds)
        mean2 = means2[point,:] * mean_bounds
        cov2 = stdev_bounds.dot(covmats2[point,:,:]).dot(stdev_bounds)
        veh1 = motionModels.initialState_normal(mean1,cov1,False)
        veh2 = motionModels.initialState_normal(mean2,cov2,False)
        out = alarms.alarm_particle(veh1, veh2, MM1, MM2, times, nrepeats)
        y_0 += [out[0]]
    y_0 = np.array(y_0)
    
    np.save(model_name+'_X.npy', X_0)
    np.save(model_name+'_Y.npy',y_0)

    
def scoreModel(truth, pred, logit):
    if logit:
        pred = 1./(1+np.exp(-pred))
    else:
        pred = np.minimum(np.maximum(pred,0.),1.)
    print "test R^2 "+str(
        1-np.sum((pred - truth)**2.)/np.sum((np.mean(truth)-truth)**2.) )
    print "AUC "+str( AUC(*ROC(truth, pred)) )
        
    
def trainModels():
    X_0 = np.load(model_name+'_X.npy')
    y_0 = np.load(model_name+'_Y.npy')
    logit = False
    
    print "mean y = "+str(np.mean(y_0))
    print "RMSR y = "+str(np.std(y_0))
    
    X_train, X_test, y_train, y_test = train_test_split(X_0,y_0,test_size=.01)
    if logit:
        y_train[y_train > .9999] = .9999
        y_train[y_train < .0001] = .0001
        y_train = np.log(y_train / (1 - y_train))
    
    print "MLP"
    mlp = MLPRegressor(hidden_layer_sizes=(150,), tol=1e-10, max_iter=500)
    mlp.fit(X_train, y_train)
    print "train R^2 "+str( mlp.score(X_train, y_train) )
    scoreModel(y_test, mlp.predict(X_test), logit)
    ff = time.time()
    mlp.predict(X_0[:100])
    print "runtime for 100 samples "+str( time.time()-ff )

    print "GBR"
    gbr = GradientBoostingRegressor(max_depth=4)# default 3
    gbr.fit(X_train,y_train)
    print "train R^2 "+str( gbr.score(X_train, y_train) )
    scoreModel(y_test, gbr.predict(X_test), logit)
    ff = time.time()
    gbr.predict(X_0[:100])
    print "runtime for 100 samples "+str( time.time()-ff )

    print "DT"
    dt = DecisionTreeRegressor(max_depth=14)
    dt.fit(X_train,y_train)
    print "train R^2 "+str( dt.score(X_train, y_train) )
    scoreModel(y_test, dt.predict(X_test), logit)
    ff = time.time()
    dt.predict(X_0[:100])
    print "runtime for 100 samples "+str( time.time()-ff )
    
    np.save(model_name+'_MLP.npy',mlp)
    
class Model():
    def __init__(self, name='MLP'):
        self.model=np.load(model_name+'_'+name+'.npy').item()
        self.stdev = np.diag(1./np.diagonal(stdev_bounds))
    def alarm_model(self, vehicle1, vehicle2, MM1, MM2, times):
        starttime = time.time()
        x = np.array([[0.]*52])
        mean1 = vehicle1.mean
        mean2 = vehicle2.mean.copy()
        mean2[:2] = mean2[:2] - mean1[:2]
        x[0,:4] = mean1[2:] / mean_bounds[2:]
        x[0,25:31] = mean2 / mean_bounds
        cov1 = self.stdev.dot(vehicle1.cov).dot(self.stdev)
        x[0,4:25] = np.concatenate((cov1[0,:],cov1[1,1:],cov1[2,2:],
                                cov1[3,3:],cov1[4,4:],cov1[5,5:]),axis=0)
        cov1 = self.stdev.dot(vehicle2.cov).dot(self.stdev)
        x[0,31:] = np.concatenate((cov1[0,:],cov1[1,1:],cov1[2,2:],
                                cov1[3,3:],cov1[4,4:],cov1[5,5:]),axis=0)
        return (self.model.predict(x)[0] , time.time() - starttime)
        
if __name__=='__main__':
    pass
    #createTruth(10**6)