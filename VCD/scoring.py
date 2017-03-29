# -*- coding: utf-8 -*-
"""
Contains methods for evaluating the quality of predicted probabilities, given
the true probabilities.
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cmx


# performance measures
def totalFPR(truth,pred,cutoff):
    """ The expected False Positive Rate given predicted probabilities and a
        cutoff value. """
    return sum(1-truth[pred>cutoff])/sum(1-truth)
def totalFNR(truth,pred,cutoff):
    """ The expected False Negative Rate given predicted probabilities and a
        cutoff value. """
    return sum(truth[pred<=cutoff])/sum(truth)
def worstFalsePositive(truth, pred, cutoff):
    """ Find the initial state for which the predicted probabilities and cutoff
        have the highest probability of False Positives."""
    if np.all(pred <= cutoff): return 0.
    return 1 - np.min(truth[pred>cutoff])
def worstFalseNegative(truth, pred, cutoff):
    """ Find the initial state for which the predicted probabilities and cutoff
        have the highest probability of False Negatives."""
    if np.all(pred > cutoff): return 0.
    return np.max(truth[pred<=cutoff])
def ROC(truth, pred):
    """ Returns the sweep of false positive and negative rates. """
    truth = truth[np.argsort(pred)]
    FP = np.cumsum(1-np.flipud(truth)) / sum(1-truth)
    FN = np.flipud(np.cumsum(truth)) / sum(truth)
    return [np.append(0.,FP), np.append(FN,0.)]
def AUC(FP, FN):
    """ Given a sweep across False Positive and Negative Rates, calculates the
        area under the Receiver Operating Characteristic. """
    fp = np.diff(FP)
    fn = np.diff(1-FN)
    fn2 = 1 - FN[:-1]
    return np.sum(fp*(fn2+fn/2.))
def zCost(truth, pred, z):
    """ Calculates the Expected Cost as defined in the paper.
    
        z = the penalty of a False Negative event.
        A False Positive event is assumed to have penalty 1. """
    # the cost of a false negative is z times the cost of a false positive
    # calculate total cost
    cutoff = 1./(z+1) # this is overapproximation in worst case
    return (sum(1-truth[pred>cutoff]) + z*sum(truth[pred<=cutoff]))/truth.shape[0]
    
    
def plotROC(truth, results, savename=None):
    """ Plots the Receiver Operating Characteristic for all alarm systems in a
    single plot. """
    plt.figure(figsize=(9.,8.))
    colors = [cmx.viridis(x) for x in np.linspace(0.,1., len(results))]
    counter = 0
    for label, result in results.items():
        preds = np.array([pred for pred, runtime in result])
        FP,FN = ROC(truth, preds)
        plt.plot(FP, 1-FN, color=colors[counter], label=label)
        counter += 1
    plt.legend()
    plt.xlabel('FPR')
    plt.ylabel('FNR')
    plt.show()
    if savename is not None:
        plt.savefig(savename+'.png', dpi=100)

        
def bigTable(truth, results, savename=None, fp_fn_vals=[], worst_vals=[],
             z_cost_vals=[]):
    """ Gathers various scores for each alarm and puts them in a table. """
    criteria = ['AUC', 'avg runtime']
    for fp_fn_val in fp_fn_vals:
        criteria += ['FNR @ cutoff='+str(fp_fn_val),
                     'FPR @ cutoff='+str(fp_fn_val)]
    for worst_val in worst_vals:
        criteria += ['worst case FN @ cutoff='+str(worst_val),
                     'worst case FP @ cutoff='+str(worst_val)]
    for z_cost_val in z_cost_vals:
        criteria += ['FNR @ z='+str(z_cost_val),
                     'FPR @ z='+str(z_cost_val),
                     'avg cost @ z='+str(z_cost_val)]
    
    results2 = {}
    for label, result in results.items():
        preds = np.array([pred for pred, runtime in result])
        runtimes = [runtime for pred, runtime in result]
        FP,FN = ROC(truth, preds)
        meanruntime = np.mean(runtimes)
        result2 = [AUC(FP,FN) , meanruntime]
        for fp_fn_val in fp_fn_vals:
            result2 += [totalFNR(truth, preds, fp_fn_val),
                        totalFPR(truth, preds, fp_fn_val)]
        for worst_val in worst_vals:
            result2 += [worstFalseNegative(truth, preds, worst_val),
                        worstFalsePositive(truth, preds, worst_val)]
        for z_cost_val in z_cost_vals:
            result2 += [totalFNR(truth, preds, 1./(1+z_cost_val)),
                        totalFPR(truth, preds, 1./(1+z_cost_val)),
                        zCost(truth, preds, z_cost_val)]
        results2[label] = result2
    results2 = pd.DataFrame(results2, index=criteria).transpose()
    if savename is not None:
        results2.to_csv(savename+'.csv', header=True, index=True)
    return results2
