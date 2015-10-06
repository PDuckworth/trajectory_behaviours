#!/usr/bin/env python

"""Use the same trajectories as the qualitative approach, and
attempt to fit a fully metric EM learning approach"""

from __future__ import division
import os, sys
import cPickle as pickle
import datetime, time
import numpy as np
import math


def load_data(file, vis=None):
    ##Get date from pickle
    with open(file, "rb") as f:
        data = pickle.load(f)
    print "number of trajectories loaded = ", len(data)

    max_number_of_poses = 0
    for cnt, (uuid, poses) in enumerate(data.items()):
        if vis: print "%s: %s Poses: %s" % (cnt, uuid, len(poses))
        if len(poses) > max_number_of_poses:
            max_number_of_poses = len(poses)

    print "max poses: %s" % max_number_of_poses

    return data, max_number_of_poses


def fix_data_length(data, max_number_of_poses=None, file=None):
    if max_number_of_poses is not None:

        new_data = {}
        for uuid, poses in data.items():
            diff = max_number_of_poses - len(poses)
            new_poses = [poses[-1]]*diff
            poses.extend(new_poses)
            new_data[uuid] = poses

        if file is not None:
            pickle.dump(new_data, open(file, "wb"))
            print "saved: %s" %file
        else:
            return new_data

def discretise_data(data, bin_size):

    data_ret = {"x":[], "y":[], "angles":[], "velocities":[]}
    print "number of subject ids: ", len(data)

    discretised_data = {}
    for cnt, (uuid, poses) in enumerate(data.items()):

        xs, ys, angles, velocities = [], [], [], []   #valid x and y's with positive velocity
        p = []
        for x_, y_, z_ in poses:

            x = int(x_ / float(bin_size))
            y = int(y_ / float(bin_size))
            p.append((x, y))

        discretised_data[uuid] = p
    return discretised_data


def E_init_step(params, vis=False):
    """
    Initialises the expectation matrix with a unimodal dist - for each trajectory
    return: matrix of Expected values, shape (NxM) = (#of trajectories, #of motion behaviours)
    """
    num_of_motions = params["num_of_motions"]
    num_of_trajs = params["num_of_trajs"]

    E_mat = np.zeros(num_of_motions*num_of_trajs).reshape(num_of_trajs, num_of_motions)

    for i in xrange(0, num_of_trajs):

        random_vec = np.random.rand(num_of_motions)
        #sum_ = sum(random_vec)
        dist_vec = random_vec / sum(random_vec)
        E_mat[i,:] = dist_vec

        if vis:
            print "motion m = %s of %s " % (i, num_of_motions)
            print random_vec, sum(random_vec)
            print dist_vec
            print E_mat

    #print E_mat.sum(axis=1)
    return E_mat

def E_step(E_mat, data, mu, params, sigma=None, vis=False ):
    """
    returns: matrix of Expected values, shape (NxM) = (#of trajectories, #of motion behaviours)
    """
    num_of_motions = params["num_of_motions"]
    num_of_trajs = params["num_of_trajs"]
    num_of_timepoints = params["num_of_timepoints"]

    if sigma is None: sigma = 1

    (uuids, all_poses) = data

    for motion in xrange(0, num_of_motions):
        if vis: print "\nmotion: ", motion
        for cnt, (uuid, poses) in enumerate(zip(uuids, all_poses)):
            if cnt>=(num_of_trajs): continue

            if vis: print "\nsubject: ", cnt
            u = [ pose[0] for pose in poses]
            v = mu[motion]
            product_these = 1

            for t in xrange(0, num_of_timepoints):
                dist = (u[t] - v[t])**2
                product_this = math.exp((-1/float(2 * sigma**2) * dist))
                if vis: print "x = %s, mu_m[t] = %0.3f, dist = %0.3f, exp(-1/2sigma**2*dist): %0.5f"\
                      % (u[t], v[t], dist, product_this)
                product_these *= product_this

            if vis: print product_these
            E_mat[cnt, motion] = product_these
        if vis: print E_mat[:, motion]
        print "here: \n", E_mat


    #columns of E_mat must sum to 1, as they are likelihoods:
    for traj in xrange(0, num_of_trajs):
        s = sum(E_mat[traj, :])
        E_mat[traj, :] = E_mat[traj, :] / s

    return E_mat


def M_step(mu, data, E_mat, params, vis=False):
    """
    returns: matrix of Mean values (mu), shape (MxT) = (#of motion behaviours x #of timepoints)
    """
    num_of_motions = params["num_of_motions"]
    num_of_trajs = params["num_of_trajs"]
    num_of_timepoints = params["num_of_timepoints"]

    (uuids, all_poses) = data

    for motion in xrange(0, num_of_motions):
        if vis: print "\nmotion: ", motion
        for t in xrange(0, num_of_timepoints):
            if vis: print "\n  time: ", t

            nominator, denominator = 0,0
            if vis: print "  E_mat = %s  " % E_mat[:,motion]
            for cnt, (uuid, poses) in enumerate(zip(uuids, all_poses)):
                if cnt>=(num_of_trajs): continue

                x, y = poses[t]
                if vis: print "    subject: %s \n    E = %s, poses: %s" % (cnt, E_mat[cnt, motion], poses[t])

                nominator += E_mat[cnt, motion]*x
                denominator += E_mat[cnt, motion]
                if vis: print "    nom: %s, denom %s" %(E_mat[cnt, motion]*x, E_mat[cnt, motion])

            mu[motion, t] = nominator/denominator
            if vis: print "  mu (%s, %s) = %s " % (motion, t,  mu[motion, t])
    return mu


def overall_likelihood(E_mat):
    """
    Returns the overall average likelihood of an Expectation Matrix
    """
    average_likelihood = 0
    for each_subject in E_mat:
        average_likelihood += max(each_subject)

    return average_likelihood/float(len(E_mat))


def mini_example_to_check():
    """Example with fake data
    """

    #Initialise E_mat randomly (with unimodal peak)
    E_mat = np.array([[0.6, 0.4],
                      [0.1, 0.9],
                      [0.8, 0.2]])

    #E_mat = np.array( [[0.6 ,0.4], [0.1, 0.9]] )
    print "\nE_mat^1 = \n", E_mat

    #Create some fake pose data
    fake_data = {1: [(1,1), (10,10)],
                 2: [(1,1),(5,5)],
                 3: [(4,4), (25,25)]}

    uuids = fake_data.keys()
    poses = [fake_data[i] for i in sorted(uuids)]
    data = (uuids, poses)

    params = {"num_of_motions": 2,
              "num_of_trajs": 3,
              "num_of_timepoints": 2}

    #Initialise the mu matrix
    mu = np.zeros(2*2).reshape(2, 2)

    num_of_em_iterations = 3
    for iter in xrange(1, num_of_em_iterations+1):
        print "\nNumber of Interations ", iter

        mu = M_step(mu, data, E_mat, params, vis=False)
        print "\nmu%s = \n %s" % (iter, mu)

        E_mat = E_step(E_mat, data, mu, params, vis=False)
        print "\nE_mat%s = \n %s" % (iter, E_mat)

        #Stopping Criteria:
        print "overall likelihood = ", overall_likelihood(E_mat)


if __name__ == "__main__":

    mini_example_to_check()
    sys.exit(1)

    file = '/home/strands/STRANDS/TESTING/offline_UUIDS_to_include_in_metric_roi_1'
    #Get data
    data, max_number_of_poses = load_data(file, vis=False)

    #Fix the len of all poses:
    #file = '/home/strands/STRANDS/TESTING/metric_uuid_poses_append.p'
    extended_data = fix_data_length(data, max_number_of_poses)

    #Discretise data
    print "discretising data..."
    #todo: set the bin_size the same as our Qualitative Occu Grid
    discretised_data = discretise_data(data=extended_data, bin_size=1)


    #Manage the trajectory data
    uuids = discretised_data.keys()
    poses = [discretised_data[i] for i in uuids]
    data = (uuids, poses)


    #EM iterations
    num_of_motions = 3                                              #Set M = 10
    num_of_trajs = 5        #len(discretised_data.keys())           #This is N
    num_of_timepoints = max_number_of_poses                         #This is T

    num_of_em_iterations = 3

    params = {"num_of_motions": num_of_motions,
              "num_of_trajs": num_of_trajs,
              "num_of_timepoints": num_of_timepoints,
              "num_of_em_iterations": num_of_em_iterations}
    print "params = ", params

    ##INITIALISATION STEP:
    E_mat = E_init_step(params, vis=False)
    print "Initial E_mat: \n", E_mat

    mu = np.zeros(num_of_motions*num_of_timepoints).reshape(num_of_motions, num_of_timepoints)
    print "Initial mu: \n", mu

    for i in xrange(0, num_of_em_iterations-1):
        print "i = ", i+1


        #Do the M step
        mu = M_step(mu, data, E_mat, params, vis=False)
        print "\nprint a bit of mu...:\n", mu[:,0:10]

        #Do the E step
        E_mat = E_step(E_mat, data, mu, params, vis=False)
        print "\nprint a bit of E_mat...:\n", E_mat[0:10,:]