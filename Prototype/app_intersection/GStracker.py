# -*- coding: utf-8 -*-
"""
last mod 11/13/18

multi-bernoulli mixture filter
objects: unspecified
"""
import numpy as np
import numba as nb
from prototype.app_intersection.murtyMH import Murty
#from scipy.spatial import cKDTree
import bisect
        

#@nb.jit(nopython=True)
def parallelPredict(samples, weights, predict, survival):#, time):
    predict_prep_cov = np.zeros((6,6)) # necessary for changing w/o reshape
    for k in range(samples.shape[0]):
        sample = samples[k]
        predict(sample, predict_prep_cov)#, time)
        weights[k] *= survival(sample)#, time)
    
#@nb.jit(nopython=True)
def parallelBuildMatchup(samples, weights, sample_msmt, detect, prepObject,
                         likelihood, occlude, measurements,
                         measurements_occluding, prepvars):
    n_msmts = len(measurements)
    for k in range(samples.shape[0]):
        sample = samples[k]
        prepObject(sample, *prepvars)
        exist = weights[k]
        match = sample_msmt[k]
        
        detect_prob = detect(sample)
        sample_miss_pure = 1 - detect_prob
        sample_miss_pure += detect_prob *\
                    occlude(sample, measurements_occluding)
        match[0] = exist * sample_miss_pure
        
        for msmt_idx in range(n_msmts):
            match[msmt_idx+1] = exist * detect_prob * likelihood(
                    sample, measurements[msmt_idx], *prepvars)
        #match[len(measurements)+1:] = 0


def parallelExtraUpdate(ids, new_ids, id_count, weights, new_weights,
                        miss_obj_exist_prob, new_obj_exist_prob, idxs):
    for new_idx, idx in enumerate(idxs):
        sample_idx, msmt_idx = idx
        
        if sample_idx == -1: # new object
            new_weights[new_idx] = new_obj_exist_prob[msmt_idx]
            new_ids[new_idx] = id_count
            id_count += 1
        elif msmt_idx == -1: # missing object
            new_weights[new_idx] = miss_obj_exist_prob[sample_idx]
            new_ids[new_idx] = ids[sample_idx]
        else:
            new_weights[new_idx] = 1.
            new_ids[new_idx] = ids[sample_idx]
    return id_count

#@nb.jit(nopython=True)
def parallelUpdate(samples, new_samples, sprouts, idxs,
                   measurements, prepvars, prepObject, update):
    for new_idx, idx in enumerate(idxs):
        sample_idx, msmt_idx = idx
        
        if msmt_idx == -1:
            new_samples[new_idx] = samples[sample_idx]
        
        else:
            new_sample = new_samples[new_idx]
            msmt = measurements[msmt_idx]
            if sample_idx == -1:
                sample = sprouts[msmt_idx]
            else:
                sample = samples[sample_idx]
            prepObject(sample, *prepvars)
            update(sample, msmt, new_sample, *prepvars)
                


class Tracker():
    def __init__(self, n_samples, n_hypotheses, n_features, max_n_measurements):
        self.max_ns = n_samples
        self.max_nhyp = n_hypotheses
        self.nft = n_features
        self.nmsmt = max_n_measurements
        
        self.samples = np.zeros((self.max_ns, self.nft))
        # structure for storing likelihoods
        self.sample_msmt = np.zeros((self.max_ns, self.nmsmt+1))
        self.weights = np.zeros((self.max_ns,))
        self.hypotheses = np.zeros((self.max_nhyp, self.max_ns), dtype=bool)
        self.hypothesis_weights = np.zeros((self.max_nhyp,))
        self.ids = np.zeros((self.max_ns,), dtype=np.uint8)
        self.id_count = 0
        
        self.ns = 0
        self.nhyp = 0
        
        self.new_samples = self.samples.copy()
        self.new_weights = self.weights.copy()
        self.new_hypotheses = self.hypotheses.copy()
        self.new_hypothesis_weights = self.hypothesis_weights.copy()
        self.new_ids = self.ids.copy()
        self.new_ns = 0
        self.new_nhyp = 0
        
        self.diagnostic_secondhypweight = []
        self.diagnostic_ns = []
        
        self.fnDetect = None
        self.fnOcclude = None
        self.fnLikelihood = None
        self.fnPrepObject = None
        self.prep = None
        self.fnPredict = None
        self.fnSurvival = None
        self.fnUpdate = None
        self.fnEntryFromMsmt = None
        self.fnDebug = None


    def reset(self):
        self.ns = 0
        self.weights[:] = 0
        self.nhyp = 1
        self.hypothesis_weights[:] = 0
        self.hypothesis_weights[0] = 1
        self.hypotheses[0] = 0
        self.id_count = 0
        
    def predict(self):
        # do this kind of stuff in a separate function
        # so that it can be numba'd later
        parallelPredict(self.samples[:self.ns], self.weights[:self.ns],
                        self.fnPredict, self.fnSurvival)
            
#    def entryPoisson(self, entry_samples, entry_cardinalities):
#        for entry_sample, entry_weight in zip(entry_samples, entry_cardinalities):
#            min_exist = np.argmin(self.sprout_weights)
#            if self.sprout_weights[min_exist] < entry_weight:
#                self.sprouts[min_exist] = entry_sample
#                self.sprout_weights[min_exist] = entry_weight
            
    def entryBernoulli(self, entry_samples, entry_cardinalities):
        for entry_sample, entry_weight in zip(entry_samples, entry_cardinalities):
            min_exist = np.argmin(self.weights)
            if self.weights[min_exist] < entry_weight:
                self.samples[min_exist] = entry_sample
                self.weights[min_exist] = entry_weight
                self.hypotheses[min_exist] = True
            
    def update(self, measurements, fp_msmt_probs, new_msmt_probs,
               measurements_occluding, merge_situations):
        n_msmts = len(measurements)
        assert n_msmts <= self.nmsmt
        
        n_merge_situations = len(merge_situations)
            
        ## gather object-measurement likelihoods
        sample_msmt = self.sample_msmt[:self.ns, :n_msmts+1]
        parallelBuildMatchup(self.samples[:self.ns], self.weights[:self.ns],
                             sample_msmt, self.fnDetect, self.fnPrepObject,
                             self.fnLikelihood, self.fnOcclude,
                             measurements, measurements_occluding, self.prep)
        # for updating after data association
        miss_obj_exist_prob = sample_msmt[:self.ns,0] / (
                            1-self.weights[:self.ns] + sample_msmt[:self.ns,0])
        sample_msmt[:self.ns,0] += 1 - self.weights[:self.ns]
        sample_msmt = -np.log(sample_msmt) # DA uses log-probability
        
        # miss measurement and new object terms
        miss_msmt = new_msmt_probs + fp_msmt_probs
        new_obj_exist_prob = new_msmt_probs / miss_msmt # for update after DA
        if self.fnEntryFromMsmt is not None:
            # make potential new object for each measurement
            msmt_entries = np.zeros((n_msmts, self.samples.shape[1]))
            for msmt_idx, msmt in enumerate(measurements):
                entry_sample = self.fnEntryFromMsmt(msmt)
                msmt_entries[msmt_idx] = entry_sample
                self.fnPrepObject(entry_sample, *self.prep)
                msmt_prob = self.fnLikelihood(entry_sample, msmt, *self.prep)
                if msmt_prob < 1e-30:
                    print msmt
                    print entry_sample[:5]
                    print self.prep[0]
                    print self.prep[1]
                    raise Exception
                miss_msmt[msmt_idx] *= msmt_prob
        else:
            raise Exception("needs entryFromMsmt function, other methods not implemented")
        miss_msmt = -np.log(miss_msmt) # DA uses log-probability
        
        
        ## data association
        # call murty's algorithm on each old hypothesis
        murty = Murty()
        wherehyp = [] # ordered indices of included samples for each hypothesis
        log_hypothesis_weights = -np.log(self.hypothesis_weights[:self.nhyp])
        for old_hyp in range(self.nhyp):
            old_hypothesis = self.hypotheses[old_hyp, :self.ns]
            hyp_msmt = sample_msmt[old_hypothesis]
            wherehyp.append(np.where(old_hypothesis)[0])
            hyp_obj_msmt = hyp_msmt[:,1:]
            for merge_indexing in merge_situations:
                this_obj_msmt = hyp_obj_msmt[:,merge_indexing].copy()
                this_miss_msmt = miss_msmt[merge_indexing].copy()
                murty.add(this_obj_msmt, hyp_msmt[:,0], this_miss_msmt, 
                          log_hypothesis_weights[old_hyp])
        assert len(murty.Q.queue) > 0, this_miss_msmt
        # find the most likely hypotheses sequentially
        # build up a list of (old sample, msmt) tuples for the new samples
        # keep most likely max_ns samples
        # cut off the hypothesis if the finite-sample approximation is too unlikely
        new_hyp_weights = []
        old_to_new = [] # using multiple lists instead of dictionary for
                        # convenience with numpy's argpartition
        sample_to_hypotheses = []
        total_weight = []
        for new_hyp in range(self.max_nhyp):
            rejected_weight = 0.
            old_to_new_backup = [ele for ele in old_to_new]
            sample_to_hypotheses_backup = [ele for ele in sample_to_hypotheses]
            # which current hypothesis has the next best new hypothesis
            cost, old_hyp, sol_matches = murty.get()
            if cost == np.inf: # there aren't any more valid hypotheses
                assert new_hyp > 0, "no hypotheses"
                break
            # figure out prior hypothesis id and merge situation id
            merge_indexing = merge_situations[old_hyp % n_merge_situations]
            obj_indexing = wherehyp[old_hyp // n_merge_situations]
            if new_hyp == 0: # first hypothesis, therefore best new hypothesis
                # avoid floating-point issues by taking out a constant
                best_cost = cost
            cost = np.exp(best_cost - cost)
            for match in sol_matches:
                obj_match, msmt_match = match
                if msmt_match >= 0: msmt_match = merge_indexing[msmt_match]
                if obj_match >= 0: obj_match = obj_indexing[obj_match]
                match = obj_match, msmt_match
                if obj_match == -1:
                    new_weight = new_obj_exist_prob[msmt_match]
                elif msmt_match == -1:
                    new_weight = miss_obj_exist_prob[obj_match]
                else:
                    new_weight = 1
                new_total_weight = new_weight * cost
                prev_idx = bisect.bisect(old_to_new, match) - 1
                if len(old_to_new) == 0 or prev_idx < 0 or\
                        old_to_new[prev_idx] != match:
                    # this obj-msmt match wasn't found in any previous hypotheses
                    old_to_new.append(match)
                    sample_to_hypothesis = [ele for ele in [False] * self.max_nhyp]
                    sample_to_hypothesis[new_hyp] = True
                    sample_to_hypotheses.append(sample_to_hypothesis)
                    total_weight.append(new_total_weight)
                else:
                    # this obj-msmt match is already present
                    sample_to_hypotheses[prev_idx][new_hyp] = True
                    total_weight[prev_idx] += new_total_weight
                assert len(old_to_new) == len(sample_to_hypotheses), match
                assert len(old_to_new) == len(total_weight), match
            if len(old_to_new) > self.max_ns:
                partitioned = np.argpartition(total_weight, -self.max_ns)
                rejected_weight = sum(total_weight[idx] for idx in
                                      partitioned[:-self.max_ns])
                if new_hyp > 0 and rejected_weight > .1:
                    # sample loss is too significant, better to ignore hypothesis
                    old_to_new = old_to_new_backup
                    sample_to_hypotheses = sample_to_hypotheses_backup
                    break
                # resort to match ordering for faster insertions in the next round
                keep = sorted(partitioned[-self.max_ns:], key=lambda idx: old_to_new[idx])
                old_to_new = [old_to_new[idx] for idx in keep]
                sample_to_hypotheses = [sample_to_hypotheses[idx] for idx in keep]
                total_weight = [total_weight[idx] for idx in keep]
                assert len(old_to_new) == len(sample_to_hypotheses), keep
                assert len(old_to_new) == len(total_weight), keep
            new_hyp_weights.append(cost)
            new_hyp += 1
        murty.close()
        self.new_hypotheses[:,:len(old_to_new)] = np.array(sample_to_hypotheses).T
        
        ## update new structures
        self.new_ns = len(old_to_new)
        self.new_nhyp = new_hyp
        samplemap = np.array(old_to_new)
#        print samplemap
        self.id_count = parallelExtraUpdate(self.ids, self.new_ids, self.id_count,
                                            self.weights, self.new_weights,
                                            miss_obj_exist_prob, new_obj_exist_prob,
                                            samplemap)
        # actual update
        parallelUpdate(self.samples, self.new_samples, msmt_entries,
                       samplemap, measurements, self.prep, self.fnPrepObject, self.fnUpdate)
        # normalize hypothesis weights
        new_hyp_weights = np.array(new_hyp_weights)
#        print new_hyp_weights
        #max_weight = min(new_hyp_weights)
        #new_hyp_weights = np.exp(-new_hyp_weights + max_weight)
        total_probability = np.sum(new_hyp_weights)
        new_hyp_weights /= total_probability
        self.new_hypothesis_weights[:new_hyp] = new_hyp_weights
        # total probability, can be used for debugging or outlier detection
        total_probability_nlog = best_cost - np.log(total_probability)
        print total_probability_nlog
            
        
    # set current values to updated ones
    def resolveUpdate(self):
        temp_pointer = self.samples
        self.samples = self.new_samples
        self.new_samples = temp_pointer
        temp_pointer = self.ids
        self.ids = self.new_ids
        self.new_ids = temp_pointer
        temp_pointer = self.weights
        self.weights = self.new_weights
        self.new_weights = temp_pointer
        temp_pointer = self.hypotheses
        self.hypotheses = self.new_hypotheses
        self.new_hypotheses = temp_pointer
        temp_pointer = self.hypothesis_weights
        self.hypothesis_weights = self.new_hypothesis_weights
        self.new_hypothesis_weights = temp_pointer
        self.ns = self.new_ns
        self.nhyp = self.new_nhyp        
        
    ## not implemented for MBM
    def prune(self):
        # let's try a prune for hypotheses, rather than samples
        for hyp_i in xrange(self.nhyp):
            for hyp_j in xrange(hyp_i):
                different_samples = np.logical_xor(self.hypotheses[hyp_i],
                                                    self.hypotheses[hyp_j])
                difference_sum = np.sum(self.weights[different_samples])
                if difference_sum < .05: # these hypotheses are basically the same
                    print("pruning with difference {:.4f}".format(difference_sum))
                    self.hypothesis_weights[hyp_i] += self.hypothesis_weights[hyp_j]
                    self.hypothesis_weights[hyp_j] = 0
#        prunesamples = orderForPrune(self.samples)
#        prunable = self.weights > 1e-3/self.ns
#        for obj in range(self.nobj+1):
#            prune_obj = np.where(prunable & (self.ids == obj))[0]
#            if prune_obj.shape[0] < 2: continue
#            kdmeans = prunesamples[prune_obj]
#            for pair_id1, pair_id2 in cKDTree(kdmeans).query_pairs(1., eps=1.):
#                weight1 = self.weights[prune_obj[pair_id1]]
#                weight2 = self.weights[prune_obj[pair_id2]]
#                if weight1 > weight2:
#                    self.weights[prune_obj[pair_id1]] = weight1+weight2
#                    self.weights[prune_obj[pair_id2]] = 0
#                else:
#                    self.weights[prune_obj[pair_id2]] = weight1+weight2
#                    self.weights[prune_obj[pair_id1]] = 0
#        #n_meaningful_samples = np.sum(prunable)
#        #n_meaningful_samples_new = np.sum(existences > 1e-3/nsamples)
#        #print "prune {:d}/{:d}".format(n_meaningful_samples_new, n_meaningful_samples)
        
        
    def report(self, report = lambda x: x, cutoff=.5):
        include = self.hypotheses[0,:] & (self.weights > cutoff)
        return np.concatenate(( self.ids[include,None],
                                np.ones((np.sum(include),1)),
                                report(self.samples[include]) ), axis=1)
        
    def earlyReport(self, report = lambda x: x, cutoff=.5):
        include = self.new_hypotheses[0,:] & (self.new_weights > .5)
        return np.concatenate((self.new_ids[include,None],
                               self.new_samples[include]), axis=1)
        
    def debug(self):
        assert self.nhyp > 0
        assert not np.any(np.isnan(self.samples))
        assert not np.any(np.isnan(self.weights))
        assert np.all(self.hypothesis_weights >= 0)
        assert np.all(self.weights >= 0)
        assert np.all(self.weights[self.ids>0] <= 1)
        if self.fnDebug is not None:
            self.fnDebug(self.samples[self.weights > 1e-3])
        
        assert self.new_nhyp > 0
        assert not np.any(np.isnan(self.new_samples))
        assert not np.any(np.isnan(self.new_weights))
        assert np.all(self.new_hypothesis_weights >= 0)
        assert np.all(self.new_weights >= 0)
        assert np.all(self.new_weights[self.new_ids>0] <= 1)
        if self.fnDebug is not None:
            self.fnDebug(self.new_samples[self.new_weights > 1e-3])
