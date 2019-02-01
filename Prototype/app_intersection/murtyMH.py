#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
last mod 10/25/18
"""

import numpy as np
from prototype.app_intersection.jv import LAPJVinit, LAPJVfinish
from Queue import PriorityQueue, Empty

inf = 1e10 # sufficiently high float that will not cause NaNs in arithmetic

class Murty():
    def __init__(self):
        self.Q = PriorityQueue()
        self.nhyp = 0
        self.cnt = 0
        self.inputs = []
        
    def add(self, P, miss_X, miss_Y, hypcost):
        m,n = P.shape
        if m == 0 or n == 0:
            if m == 0 and n == 0:
                self.inputs.append(None)
                self.nhyp += 1
                return
            v = np.zeros((m+n,))
            C = sum(miss_Y)
            if C >= inf:
                return
            problem = (C, self.nhyp, 0, -1, range(m), range(n), [], np.arange(m+n),
                       v, [])
            self.Q.put(problem)
            self.inputs.append((P, miss_X, miss_Y, hypcost))
            self.nhyp += 1
            return
            
        c = np.zeros((m+n,m+n))
        c[:m,:n] = P
        c[:m,n:] = inf
        c[range(m), range(n,m+n)] = miss_X
        c[m:,:n] = inf
        c[range(m,m+n), range(n)] = miss_Y
    
        y = np.argmin(c, axis=0).astype(int)
        v = c[y, range(m+n)].copy()
        x = np.zeros((m+n,), dtype=int) - 1
        free = LAPJVinit(c, x, y, v)
        if len(free) > 0:
            LAPJVfinish(c, x, y, v, free)
            
        C = np.sum(c[range(m+n), x]) + hypcost
        fixed = []
        reduced_x = range(m)
        reduced_y = range(n)
        eliminate_i = -1
        eliminated = []
        
        self.Q.put((C, self.nhyp, self.cnt, eliminate_i, reduced_x, reduced_y,
               eliminated, x, v, fixed))
        self.inputs.append((P, miss_X, miss_Y, hypcost))
        self.nhyp += 1
        
    def get(self):
        eliminate_i = 0
        while eliminate_i >= 0:
            self.cnt += 1
            assert self.cnt < 10000
            
            try:
                problem = self.Q.get(block=False)
            except Empty: return np.inf, -1, ()
            C, hyp, _, eliminate_i, reduced_x, reduced_y, eliminated, x, v, fixed = problem
            
            # get reduced cost matrix
            P, miss_X, miss_Y, hypcost = self.inputs[hyp]
            m = len(reduced_x) # number of rows in reduced problem
            n = len(reduced_y) # number of columns in reduced problem
            c = np.zeros((m+n,m+n))
            c[:m,:n] = P[reduced_x,:][:,reduced_y]
            c[:m,n:] = inf
            c[range(m), range(n,m+n)] = miss_X[reduced_x]
            c[m:,:n] = inf
            c[range(m,m+n), range(n)] = miss_Y[reduced_y]
            for i,j in eliminated:
                c[i,j] = inf
            
            if eliminate_i >= 0:
                # unsolved problem
                # reconstruct y
                y = np.argsort(x).astype(int)
                # eliminate the selected match
                j = x[eliminate_i]
                x[eliminate_i] = -1
                y[j] = -1
                c[eliminate_i,j] = inf
                eliminated.append((eliminate_i, j))
                # solve
                LAPJVfinish(c, x, y, v, [eliminate_i])
                Cnew = sum(c[range(m+n), x])
                Cnew += sum((P[i,j] if j>=0 else miss_X[i]) if i>=0 else miss_Y[j]
                            for i,j in fixed)
                Cnew += hypcost
                # check that this is a valid solution - can be deleted to save time
                assert all(y[j]==i for i,j in enumerate(x))
                # check that the lower bound was in fact a lower bound - "  "
                assert C - Cnew < 1e-4
                if Cnew < inf:
                    # put back in queue, will only send when it is the best solution
                    self.Q.put((Cnew, hyp, self.cnt, -1, reduced_x, reduced_y,
                                eliminated, x, v, fixed))
                
        # problem already solved, return solution
        # combine matches from reduced matrix with previously fixed matches
        output = [(reduced_x[i] if i<m else -1, reduced_y[j] if j<n else -1)
                    for i,j in enumerate(x) if i<m or j<n]
        output = tuple(sorted(fixed + output))
    
        ## create problems that don't include this solution
        # move the auxiliary matches around, because they don't matter
        # in this particular order they reduce easily
        for i,j in enumerate(x[:m]):
            if j < n:
                x[j+m] = i+n
        # calculate slack (cost minus dual variables)
        u = c[range(m+n), x] - v[x]
        slack = c - u[:,None] - v[None,:]
#            assert not np.any(np.isnan(slack))
#            assert np.all(slack >= -1e-5)
        includex = [i for i,j in enumerate(x) if i<m or j<n]
        includey = [j for i,j in enumerate(x) if i<m or j<n]
        allx = np.arange(m+n)
        ally = np.arange(m+n)
        slack[includex, includey] = inf
        while m > 0 and n > 0:
            # get the lower bound for each removed match
            # don't include the aux submatrix
            minslack_X = np.min(slack[includex,:][:,ally], axis=1)
            minslack_Y = np.min(slack[allx,:][:,includey], axis=0)
            minslack_X = np.maximum(minslack_X, minslack_Y)
            # select the highest-cost subproblem next
            # optimization 3 from MillerStoneCox
            next_i = np.argmax(minslack_X)
            C_lowerbound = C + minslack_X[next_i]
            includex.pop(next_i)
            includey.pop(next_i)
            if next_i < m:
                next_j = x[next_i]
            else:
                next_j = [j for j in x[m:] if j<n][next_i-m]
                next_i = m + next_j
            if C_lowerbound < inf:
                # add to queue, unsolved (optimization 2 from MillerStoneCox)
                self.Q.put((C_lowerbound, hyp, self.cnt, next_i,
                            list(reduced_x), list(reduced_y), eliminated,
                            x, v, list(fixed)))
                self.cnt += 1
            
            # make reduced matrix for next successive problem
            # reducing the aux submatrix along with the main submatrix
            if next_i < m and next_j < n:
                # reduce two rows and two columns
                # one standard and one auxiliary for each
                next_i_aux = next_j + m
                next_j_aux = next_i + n
                reduce_i = range(next_i) + range(next_i+1, next_i_aux) +\
                            range(next_i_aux+1, m+n)
                reduce_j = range(next_j) + range(next_j+1, next_j_aux) +\
                            range(next_j_aux+1, m+n)
                x = x[reduce_i].copy()
                x[x > next_j_aux] -= 1
                x[x > next_j] -= 1
                eliminated = [(i - (i>next_i_aux) - (i>next_i),
                               j - (j>next_j_aux) - (j>next_j))
                              for i,j in eliminated if
                                i!=next_i and i!=next_i_aux and
                                j!=next_j and j!=next_j_aux]
                fixed_next_i = reduced_x.pop(next_i)
                fixed_next_j = reduced_y.pop(next_j)
                fixed.append((fixed_next_i, fixed_next_j))
                m -= 1
                n -= 1
            elif next_i < m:
                # reduce a row and a corresponding auxiliary column
                reduce_i = range(next_i) + range(next_i+1, m+n)
                reduce_j = range(next_j) + range(next_j+1, m+n)
                x = x[reduce_i].copy()
                x[x > next_j] -= 1
                eliminated = [(i - (i>next_i), j - (j>next_j))
                              for i,j in eliminated if
                                i!=next_i and j!=next_j]
                fixed_next_i = reduced_x.pop(next_i)
                fixed.append((fixed_next_i, -1))
                m -= 1
            elif next_j < n:
                # reduce a column and corresponding auxiliary row
                reduce_i = range(next_i) + range(next_i+1, m+n)
                reduce_j = range(next_j) + range(next_j+1, m+n)
                x = x[reduce_i].copy()
                x[x > next_j] -= 1
                eliminated = [(i - (i>next_i), j - (j>next_j))
                              for i,j in eliminated if
                                i!=next_i and j!=next_j]
                fixed_next_j = reduced_y.pop(next_j)
                fixed.append((-1, fixed_next_j))
                n -= 1
            else:
                raise Exception
            v = v[reduce_j].copy()
            allx = allx[reduce_i].copy()
            ally = ally[reduce_j].copy()
            
        return C, hyp, output
        
    def close(self):
        del self.Q
        
        
    
    
    
if __name__ == '__main__':
    from time import time
    np.random.seed(4)
    
    M = 50
    N = 50
    n_solutions = 25
    n_repeats = 51
    
    totaltime = 0.
    allcosts = []
    for repeat in range(n_repeats):
        # uniform
        P = np.random.rand(M,N)
        # exponential
        #P = -np.log(P)
        #P.tofile('test{:2d}.csv'.format(repeat))
        
        # consider missing rows and columns
        #miss_X = P[1:,0]
        #miss_Y = P[0,1:]
        #P = P[1:,1:]
        # don't allow missing rows or columns, will only have solutions if M==N
        miss_X = np.zeros((M,)) + inf
        miss_Y = np.zeros((N,)) + inf

        costs = []  
        
        totaltime -= time()
        # actual operation here!
        mm = Murty()
        mm.add(P, miss_X, miss_Y, 0.)
        for k in range(n_solutions):
            C,H,S = mm.get()
            costs.append(C)
        totaltime += time()
        if repeat == 0: totaltime = 0.
        
        #np.save('msc{:2d}.npy'.format(repeat), costs)
#        np.savetxt('m{:d}fast.txt'.format(repeat),S, fmt='%d')
    n_repeats -= 1
    print(totaltime * (1000. / n_repeats)) # average runtime in milliseconds
