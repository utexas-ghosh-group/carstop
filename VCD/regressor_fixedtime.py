""" This experiment was not presented in the paper. It simulated two normally
distributed vehicles in a single time frame, then developed a model for the
probability of collision in that time frame. This could be used in tandem with
movement predictions, for instance the unscented transform, to more quickly predict
collisions. While its runtime was excellent, its accuracy was poor despite being
trained on 17x10^6 examples. I suspect that training a model that is both accurate
and generalizable (that works on multiple scenarios, not just the one simulated
in training) will require a great deal of computational resources.
"""

from collisionCheck import check as collisionCheck
import numpy as np
from sklearn.model_selection import train_test_split
from scoring import AUC,ROC
from sklearn.neural_network import MLPRegressor
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.tree import DecisionTreeRegressor
import time

model_name = 'step'

xy_bound = 15
std_xy_bound = 10
std_angle_bound = np.pi / 3**.5

def rectify(theta):
    theta[theta > np.pi] -= 2*np.pi
    theta[theta > np.pi] -= 2*np.pi
    theta[theta < -np.pi] += 2*np.pi
    theta[theta < -np.pi] += 2*np.pi
    return theta

def createTruth(npoints, nrepeats=3000):
    """
    Generates a range of initial states for both vehicles, then simulates from
    each one a certain number of times.
    """
    ## generating a grid of features
    ndim = 8
    res = int(npoints**(1./ndim))
    npoints = res**ndim
    X_0 = np.empty((npoints, ndim))
    x_0 = np.linspace(-1.,1.,res)
    for feature in range(4):
        X_0[:,feature] = np.repeat(np.tile(x_0, res**feature), res**(ndim-feature-1))
    x_0 = np.linspace(0.,1.,res)
    for feature in range(4,8):
        X_0[:,feature] = np.repeat(np.tile(x_0, res**feature), res**(ndim-feature-1))
    ## randomly generating features
#    X_0 = np.random.uniform(-1,1,size=(npoints,4))
#    X_0 = np.append(X_0, np.random.uniform(0,1,size=(npoints,4)),axis=1)
    
    egocar = np.zeros((nrepeats,3))
    altcar = np.zeros((nrepeats,3))
    samples = np.random.normal(size=(4,nrepeats))
    
    y_0 = np.empty((npoints,))
    for point in range(npoints):
        feats = X_0[point,:]
        altcar[:,0] = feats[0] * xy_bound + std_xy_bound * feats[4] * samples[0]
        altcar[:,1] = feats[1] * xy_bound + std_xy_bound * feats[5] * samples[1]
        angle = feats[2] * np.pi + std_angle_bound * feats[6] * samples[2]
        egocar[:,2] = rectify(angle)
        angle = feats[3] * np.pi + std_angle_bound * feats[7] * samples[3]
        altcar[:,2] = rectify(angle)
        y_0[point] = np.sum(collisionCheck(egocar, altcar))/nrepeats
    
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
    
    X_train, X_test, y_train, y_test = train_test_split(X_0,y_0,test_size=.001)
    if logit:
        y_train[y_train > .9999] = .9999
        y_train[y_train < .0001] = .0001
        y_train = np.log(y_train / (1 - y_train))
    
    ## multi-layer perceptron
    print("MLP")
    mlp = MLPRegressor(hidden_layer_sizes=(100,4), tol=1e-10, max_iter=500)
    mlp.fit(X_train, y_train)
    print( "train R^2 {:.3f}".format( mlp.score(X_train, y_train) ))
    scoreModel(y_test, mlp.predict(X_test), logit)
    ff = time.time()
    mlp.predict(X_0[:100])
    print( "runtime for 100 samples: {:.1e}".format( time.time()-ff ))

    ## gradient boosting, a slow but highly regarded ensemble regressor
    print("GBR")
    gbr = GradientBoostingRegressor(max_depth=4)# default 3
    gbr.fit(X_train,y_train)
    print( "train R^2 {:.3f}".format( gbr.score(X_train, y_train) ))
    scoreModel(y_test, gbr.predict(X_test), logit)
    ff = time.time()
    gbr.predict(X_0[:100])
    print( "runtime for 100 samples: {:.1e}".format( time.time()-ff ))

    ## regression trees, very fast in training and use, but easily overfit data
    print("DT")
    dt = DecisionTreeRegressor(max_depth=20)
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
        self.bin = np.zeros((0,8))
        
    def alarm(self, vehicle1, vehicle2, MM1, MM2, times):
        """input and output match that of the functions in alarms.py"""
        MM1weights = MM1.weights
        MM2weights = MM2.weights
        
        starttime = time.time()
        v1 = vehicle1.utpoints
        v2 = vehicle2.utpoints
        features = np.empty((len(times),8))

        for j,dtime in enumerate(times):
            v1 = MM1.UTstep(v1, dtime)
            v2 = MM2.UTstep(v2, dtime)
            xy1 = MM1.fullToXY(v1)
            xy2 = MM2.fullToXY(v2)
            x1mean = MM1weights.dot(xy1[:,0])
            y1mean = MM1weights.dot(xy1[:,1])
            cosmean = MM1weights.dot(np.cos(xy1[:,2]))
            sinmean = MM1weights.dot(np.sin(xy1[:,2]))
            th1mean = np.arctan2(sinmean, cosmean)
            x1std = MM1weights.dot((xy1[:,0] - x1mean)**2.)
            y1std = MM1weights.dot((xy1[:,1] - y1mean)**2.)
            th1std = MM1weights.dot(rectify(xy1[:,2] - th1mean)**2.)**.5
            x2mean = MM2weights.dot(xy2[:,0])
            y2mean = MM2weights.dot(xy2[:,1])
            cosmean = MM2weights.dot(np.cos(xy2[:,2]))
            sinmean = MM2weights.dot(np.sin(xy2[:,2]))
            th2mean = np.arctan2(sinmean, cosmean)
            x2std = MM2weights.dot((xy2[:,0] - x2mean)**2.)
            y2std = MM2weights.dot((xy2[:,1] - y2mean)**2.)
            th2std = MM2weights.dot(rectify(xy2[:,2] - th2mean)**2.)**.5
            features[j,:] = [ (x2mean - x1mean) / xy_bound,
                              (y2mean - y1mean) / xy_bound,
                              th1mean / np.pi,
                              th2mean / np.pi,
                              (x1std+x2std)**.5 / std_xy_bound,
                              (y1std+y2std)**.5 / std_xy_bound,
                              th1std / std_angle_bound,
                              th2std / std_angle_bound ]
        pred = self.model.predict(features)
        pred = np.max(pred)
        endtime = time.time() - starttime
        self.bin = np.append(self.bin, features, axis=0)
        return (min(max(pred,0.),1.) , endtime)
      
        
""" uncomment createTruth or trainModels() to easily run from the command line
    comment it again before running the main simulator! """
if __name__=='__main__':
    pass
    #createTruth(2*10**7, 10000)
    #trainModels()