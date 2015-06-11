#!/usr/bin/env python

"""compute_spatial_relations.py: Computes spatial relations."""

__author__ = "Paul Duckworth"
__copyright__ = "Copyright 2015, University of Leeds"

import rospy
import pymongo
import os, sys, time
import logging
import argparse
import itertools
import getpass

import numpy as np
from scipy import spatial
import cPickle as pickle

from geometry_msgs.msg import Pose, Quaternion
from human_trajectory.msg import Trajectory, Trajectories
from soma_trajectory.srv import TrajectoryQuery, TrajectoryQueryRequest, TrajectoryQueryResponse
from soma_geospatial_store.geospatial_store import *

import novelTrajectories.config_utils as util
import relational_learner.obtain_trajectories as ot
import relational_learner.trajectory_analysis as th

from relational_learner.graphs_handler import *
from novelTrajectories.traj_data_reader import *
from relational_learner.learningArea import *


def Mongodb_to_list(res):
    """Convert the EpisodeMsg into a list of episodes
       episodeMsg[]
           string obj1
           string obj1_type
           string obj2
           string obj2_type
           string spatial_relation
           int32 start_frame
           int32 end_frame
    """

    ep_list = []
    for i in res:
        ep = (str(i["obj1"]), str(i["obj1_type"]), str(i["obj2"]), \
              str(i["obj2_type"]), str(i["spatial_relation"]), \
              int(i["start_frame"]), int(i["end_frame"]))
        ep_list.append(ep)
    return ep_list


def run_all(plotting=False, episode_store='relational_episodes'):
    (directories, config_path, input_data, date) = util.get_learning_config()
    (data_dir, qsr, trajs, activity_graph_dir, learning_area) = directories
    (soma_map, soma_config) = util.get_map_config(config_path)

    gs = GeoSpatialStoreProxy('geospatial_store', 'soma')
    msg_store = GeoSpatialStoreProxy('message_store', episode_store)

    # *******************************************************************#
    #                  Regions of Interest Knowledge                     #
    # *******************************************************************#
    rospy.loginfo('Getting Region Knowledge from roslog...')
    roi_knowledge, roi_temp_list = region_knowledge(soma_map, soma_config, \
                                                    sampling_rate=10, plot=plotting)

    # Filter trajectories which were deemed noise by using only people_trajectory msg store
    # list_of_uuids, pose_data_dic = ot.filtered_trajectorys()
    uuid_pose_dict = ot.filtered_trajectorys()
    list_of_uuids = uuid_pose_dict.keys()
    print "# uuids quried in people_trajectory msg store = ", len(list_of_uuids)
    # pickle.dump(uuid_pose_dict, open(data_dir+"trajectory_dump/pose_data_dic_leeds.p",'wt'))

    # *******************************************************************#
    #              Analyse the shape of the Trajectories                 #
    # *******************************************************************#
    rospy.loginfo('Generating Heatmap of trajectories...')
    dt = th.Discretise_Trajectories(data=uuid_pose_dict, bin_size=0.1, filter_vel=1)
    dt.heatmap_run(vis=plotting, with_analysis=True)

    rospy.loginfo('Self generating interesting destination points...')
    interest_points = dt.plot_polygon(vis=plotting, facecolor='green', alpha=0.4)
    print "interesting points include:\n", interest_points


    dt.markov_chain.display_and_save(layout='nx', view=True, path=trajs)


    # *******************************************************************#
    #                  Obtain Episodes in ROI                            #
    # *******************************************************************#
    rospy.loginfo("0. Running ROI query from message_store")
    for roi in gs.roi_ids(soma_map, soma_config):
        str_roi = "roi_%s" % roi
        # if roi != '12': continue

        print '\nROI: ', gs.type_of_roi(roi, soma_map, soma_config), roi
        query = {"soma_roi_id": str(roi)}

        res = msg_store.find(query)
        all_episodes = {}
        trajectory_times = []
        for cnt, trajectory in enumerate(res):
            # print cnt
            if trajectory["uuid"] not in list_of_uuids:
                # print "UUID: %s filtered out" % str(trajectory["uuid"])
                continue
            all_episodes[trajectory["uuid"]] = Mongodb_to_list(trajectory["episodes"])
            trajectory_times.append(trajectory["start_time"])

        cnt += 1  # enumerate starts cnt from 0
        print "Total Number of Trajectories = %s." % cnt
        print "Number of Trajectories after filtering = %s." % len(all_episodes)

        if len(all_episodes) < 12:
            print "Not enough episodes in region %s to learn model." % roi
            continue

        # **************************************************************#
        #            Activity Graphs/Code_book/Histograms               #
        # **************************************************************#
        rospy.loginfo('Generating Activity Graphs')

        params, tag = AG_setup(input_data, date, str_roi)
        print "INFO: ", params, tag, activity_graph_dir

        generate_graph_data(all_episodes, activity_graph_dir, params, tag)

        # **************************************************************#
        #           Generate Feature Space from Histograms              #
        # **************************************************************#
        rospy.loginfo('Generating Feature Space')
        feature_space = generate_feature_space(activity_graph_dir, tag)

        (code_book, graphlet_book, X_source_U) = feature_space
        print "code_book length = ", len(code_book)

        # **************************************************************#
        #                    Create a similarty space                   #
        # **************************************************************#
        rospy.loginfo('Create Similarity Space')
        similarity_space = get_similarity_space(feature_space)
        dictionary_of_similarity = {}

        for i in similarity_space:
            key = np.sum(i)
            if key in dictionary_of_similarity:
                dictionary_of_similarity[key] += 1
            else:
                dictionary_of_similarity[key] = 1

                # print "similarty space matches =" #Note: Reducing +ve histogram counts to 1
                # for key, cnt in dictionary_of_similarity.items():
                # print key, cnt

        # **************************************************************#
        #                    Learn a Clustering model                   #
        # **************************************************************#
        rospy.loginfo('Learning on Feature Space')
        params, tag = AG_setup(input_data, date, str_roi)

        smartThing = Learning(f_space=feature_space, roi=str_roi, vis=False)

        pca, variable_scores = smartThing.pca_investigate_variables()

        top = 0.1  # Percentage of graphlets to analyse (make this automatic?)
        smartThing.pca_graphlets(pca, variable_scores, top)

        smartThing.kmeans(k=2)  # Can pass k, or auto selects min(penalty)

        # *******************************************************************#
        #                    Temporal Analysis                               #
        # *******************************************************************#
        rospy.loginfo('Learning Temporal Measures')
        # print "traj times = ", trajectory_times, "\n"
        smartThing.time_analysis(trajectory_times, plot=plotting)

        # Add the region knowledge to smartThing
        try:
            smartThing.methods["roi_knowledge"] = roi_knowledge[roi]
            smartThing.methods["roi_temp_list"] = roi_temp_list[roi]
        except KeyError:
            smartThing.methods["roi_knowledge"] = 0
            smartThing.methods["roi_temp_list"] = [0] * 24

        smartThing.save(learning_area)
        print "Learnt models for: "
        for key in smartThing.methods:
            print "    ", key

    print "COMPLETED LEARNING PHASE"
    return


class Offline_Learning(object):
    def learn(self, plotting=True, episode_store='relational_episodes'):
        r = run_all(plotting, episode_store)


if __name__ == "__main__":
    rospy.init_node("trajectory_learner")

    if len(sys.argv) < 2:
        rospy.logerr("usage: offlinelearning turn_on_plotting[0/1]. 0 by default.")
        plotting = False
    else:
        plotting = int(sys.argv[1])

    if len(sys.argv) < 3:
        rospy.logerr("usage: give an episode message store. `relational_episodes` by default.")
        episode_store = 'relational_episodes'
    else:
        episode_store = sys.argv[2]

    print "plotting = ", plotting
    print "message store = ", episode_store

    o = Offline_Learning()
    o.learn(plotting, episode_store)
