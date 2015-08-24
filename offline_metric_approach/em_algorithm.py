#!/usr/bin/env python

"""Use the same trajectories as the qualitative approach, and
attempt to fit a fully metric EM learning approach"""

import os, sys
import cPickle as pickle
import datetime, time
import numpy as np
import matplotlib.pyplot as plt


def load_data(file, vis=None):
    ##Get date from pickle
    with open(file, "rb") as f:
        data = pickle.load(f)
    print len(data)

    max_number_of_poses = 0
    for cnt, (uuid, poses) in enumerate(data.items()):
        if vis: print "%s: %s Poses: %s" % (cnt, uuid, len(poses))
        if len(poses) > max_number_of_poses:
            max_number_of_poses = len(poses)

    if vis: print "max poses: %s" % max_number_of_poses
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


def E_init_step(data, params):
    """
    Initialises the expectation matrix with a unimodal dist - for each traj
    returns matrix of Expected values, shape (NxM) = (#of trajectories, #of motion behaviours)
    """
    (num_of_motions, num_of_trajs, num_of_timepoints) = params
    num_of_motions = 3
    num_of_trajs = 5

    E_mat = np.zeros(num_of_motions*num_of_trajs).reshape(num_of_trajs,num_of_motions)

    for i in xrange(0, num_of_motions):
        #print "motion m = %s" %i
        random_vec = np.random.rand(5)
        #print random_vec
        sum_ = sum(random_vec)
        #print sum_
        dist_vec = random_vec / sum_
        #print dist_vec

        E_mat[:,i] = dist_vec
        #print E_mat

    #print E_mat.sum(axis=0)
    return E_mat


def Estep():
    """
    returns matrix of Expected values, shape (NxM) = (#of trajectories, #of motion behaviours)
    """



def Mstep(data, E_mat, params):
    """
    returns matrix of Mean values (mu), shape (MxT) = (#of motion behaviours, #of timepoints)
    """
    (num_of_motions, num_of_trajs, num_of_timepoints) = params
    (uuids, poses) = new_data

    #to test:
    num_of_motions = 3
    num_of_trajs = 5

    mu = np.zeros(num_of_motions*num_of_timepoints).reshape(num_of_motions, num_of_timepoints)

    for motion in xrange(0, num_of_motions):
        for t in xrange(0, num_of_timepoints):

            nominator, denominator = 0,0
            for cnt, (uuid, poses) in enumerate(zip(uuids, poses)):
                print cnt
                print E_mat
                print E_mat[motion, cnt]
                print poses
                print poses[t]
                nominator += E_mat[motion, cnt]*poses[t]
                denominator += E_mat[motion, cnt]

                """NOTE: something wrong with poses"""
            mu[motion, t] = nominator/denominator

    return mu


def discretise_data(data, bin_size):

    data_ret = {"x":[], "y":[], "angles":[], "velocities":[]}
    print "number of subject ids: ", len(data)

    discretised_data = {}
    for cnt, (uuid, poses) in enumerate(data.items()):

        xs, ys, angles, velocities = [], [], [], []   #valid x and y's with positive velocity
        print cnt, uuid

        p = []
        for x_, y_, z_ in poses:

            x = int(x_ / float(bin_size))
            y = int(y_ / float(bin_size))
            p.append((x, y))

        discretised_data[uuid] = p
    return discretised_data

if __name__ == "__main__":

    file = '/home/strands/STRANDS/TESTING/offline_UUIDS_to_include_in_metric_roi_1'
    #Get data
    data, max_number_of_poses = load_data(file, vis=False)

    #Fix the len of all poses:
    #file = '/home/strands/STRANDS/TESTING/metric_uuid_poses_append.p'
    extended_data = fix_data_length(data, max_number_of_poses)

    #Discretise data
    print "discretising data..."
    discretised_data = discretise_data(data=extended_data, bin_size=1)


    #EM iterations
    num_of_motions = 10                                             #Setting M = 10 (for now)
    num_of_trajs = len(discretised_data.keys())                     #This is N
    num_of_timepoints = max_number_of_poses                         #This is T

    num_of_em_iterations = 3

    params = (num_of_motions, num_of_trajs, num_of_timepoints)
    for i in xrange(0, num_of_em_iterations):

        if i==0:
            #initialise E step first - create a unimodal dist for each trajectory over all motion patterns
            E_mat = E_init_step(discretised_data, params)
            print E_mat
        else:

            #Do the M step
            mu = Mstep(new_data, E_mat, params)

            #Do the E step
            E_mat = Estep(discretised_data, params)