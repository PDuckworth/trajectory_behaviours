#!/usr/bin/env python

"""Activity Graph handler code"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"

import rospy
import os, sys, time
import itertools
import cPickle as pickle
import numpy as np
from relational_learner.Activity_Graph import Activity_Graph



def AG_setup(input_data, date, roi):
    params_str = (input_data['MIN_ROWS'], input_data['MAX_ROWS'], input_data['MAX_EPI'], input_data['num_cores'])
    params = []
    for x in params_str:
        params.append(int(x)) if x != 'None' else params.append(None)

    params_tag = map(str, params)
    params_tag = '_'.join(params_tag)
    tag = roi +'_'+ params_tag + date

    return params, tag


#**************************************************************#
#      Create Activity Graphs For Each Trajectory Instance     #
#**************************************************************#

def generate_graph_data(episodes, data_dir, params, tag, obj_type = 2,
                         __out=False, test=False, vis=False):
    t0 = time.time()
    cnt=0
    activity_graphs = {}

    for episodes_file in episodes:

        if __out: rospy.loginfo('Processing for graphlets: ' + episodes_file)
        episodes_list = episodes[episodes_file]

        activity_graphs[episodes_file] = Activity_Graph(episodes_list, params, obj_type)
        activity_graphs[episodes_file].get_valid_graphlets()

        if vis: graph_check(activity_graphs, episodes_file) #print activity graphs to file
        cnt+=1
        if __out: print cnt

    if test:
        rospy.loginfo('Activity Graph Data Generated')
        return activity_graphs
    else:
        AG_out_file = os.path.join(data_dir + 'activity_graphs_' + tag + '.p')
        pickle.dump(activity_graphs, open(AG_out_file,'w'))
        rospy.loginfo('Activity Graph Data Generated and saved to:\n' + AG_out_file)
    print "Done. Took %f seconds." % (time.time()-t0)
    return



def graph_check(gr, ep_file):
    """Prints dotfile to /tmp lots """
    gr[ep_file].graph2dot('/tmp/act_gr.dot', False)
    os.system('dot -Tpng /tmp/act_gr.dot -o /tmp/act_gr.png')
    print "graph: " + repr(ep_file)
    print gr[ep_file].graph

    """
    gr2 = gr[ep_file].valid_graphlets
    for cnt_, i in enumerate(gr2[gr2.keys()[0]].values()):
        i.graph2dot('/tmp/graphlet.dot', False)
        cmd = 'dot -Tpng /tmp/graphlet.dot -o /tmp/graphlet_%s.png' % cnt_
        os.system(cmd)
    """


def generate_feature_space(data_dir, tag, __out=False):
    t0 = time.time()
    AG_out_file = os.path.join(data_dir + 'activity_graphs_' + tag + '.p')
    activity_graphs = pickle.load(open(AG_out_file))

    rospy.loginfo('Generating codebook')

    code_book = {}
    code_book_set = set([])

    special_hashs=[]
    t0, t1 = 0, 0
    for episodes_file in activity_graphs:

        #for window in activity_graphs[episodes_file].graphlet_hash_cnts:     #Loop through all temporal windows, if multiple.
        window = activity_graphs[episodes_file].graphlet_hash_cnts.keys()[0]

        for ghash, graphlet in  activity_graphs[episodes_file].valid_graphlets[window].items():
            if ghash not in code_book_set:
                code_book_set.add(ghash)
                code_book[ghash] = graphlet

    code_book_hashes =[]
    code_book_hashes.extend(code_book_set)

    print "len of code book: " + repr(len(code_book_hashes))
    rospy.loginfo('Generating codebook FINISHED')

    X_source = {}
    rospy.loginfo('Generating features')
    #Histograms are Windowed dictionaries of histograms
    for cnt, episodes_file in enumerate(activity_graphs):
        if __out: print cnt, episodes_file

        histogram = activity_graphs[episodes_file].get_histogram(code_book_hashes)
        X_source[episodes_file] = histogram

        if __out and cnt ==1:
            key = activity_graphs[episodes_file].graphlet_hash_cnts.keys()[0]
            print "KEY = " + repr(key)
            print "hash counts: " + repr(activity_graphs[episodes_file].graphlet_hash_cnts[key].values())
            print "sum of hash counts: " + repr(sum(activity_graphs[episodes_file].graphlet_hash_cnts[key].values()))
            print "sum of histogram: " + repr(sum(histogram))

    rospy.loginfo('Generating features FINISHED')
    rospy.loginfo('Saving all experiment data')

    #This is: (list_of_hashes_used_to_create_hists, a_dict_of_hash_to_AG, a_dictionary_of_histograms)
    feature_space = (code_book_hashes, code_book, X_source)

    feature_space_out_file = os.path.join(data_dir + 'feature_space_' + tag + '.p')
    pickle.dump(feature_space, open(feature_space_out_file, 'w'))
    print "\nall graph and histogram data written to: \n" + repr(data_dir)
    print "Done. Took %f seconds.\n" % (time.time()-t0)
    return feature_space
