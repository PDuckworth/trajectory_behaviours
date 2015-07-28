#!/usr/bin/env python

"""visualise_qualitative_predictions.py:"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"
import os, sys
import rospy
import copy
import cPickle as pickle
import time

def visualise_qualitative_predictions():
    rospy.loginfo('Visualising qualitative predictions')
    st = time.time()

    print "time taken:", time.time() - st


if __name__ == "__main__":
    rospy.init_node('visualise_qualitative_predictions')
    plotting=True
    data_dir = '/home/strands/STRANDS/'
    file_ = os.path.join(data_dir + 'TESTING/.p')
    print file_
    reduced_test = pickle.load(open(file_, "r"))
