#!/usr/bin/env python

"""Analyse Episode store using Machine Learning techniques (offline)"""

__author__ = "Paul Duckworth"
__copyright__ = "Copyright 2015, University of Leeds"

import rospy
import pymongo
import os, sys, time
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
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
import relational_learner.graphs_handler as gh

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

    # *******************************************************************#
    #               Set up Directories and msg stores                    #
    # *******************************************************************#
    (directories, config_path, input_data, date) = util.get_learning_config()
    (data_dir, qsr, trajs, activity_graph_dir, learning_area) = directories
    (soma_map, soma_config) = util.get_map_config(config_path)

    client = pymongo.MongoClient(
        rospy.get_param("mongodb_host", "localhost"),
        rospy.get_param("mongodb_port", 62345) )
    db = client.message_store.relational_episodes_test
    gs = GeoSpatialStoreProxy('geospatial_store', 'soma')

    """
    # *******************************************************************#
    #              Analyse the shape of the Trajectories                 #
    # *******************************************************************#
    rospy.loginfo('Generating Heatmap of trajectories...')
    uuid_pose_dict = { your_key: [your_key] for your_key in best_uuids}

    dt = th.Discretise_Trajectories(data=uuid_pose_dict, bin_size=0.2, filter_vel=1, verbose=False)
    dt.heatmap_run(vis=plotting, with_analysis=plotting)

    rospy.loginfo('Generating endpoint/destination points...')
    interest_points = dt.plot_polygon(vis=plotting, facecolor='green', alpha=0.4)
    print "interesting points include:\n", interest_points
    dt.markov_chain.display_and_save(layout='nx', view=True, path=trajs)

    """

    # *******************************************************************#
    #                 Get a Test Set of Episodes                         #
    # *******************************************************************#
    #Use the final week of data as a test set
    t1 = datetime.datetime(2015, 6, 5, 00, 0, 0, 000000)
    t2 = datetime.datetime(2015, 6, 12, 00, 0, 0, 000000)
    start_seconds = (t1-datetime.datetime(1970,1,1)).total_seconds()
    end_seconds = (t2-datetime.datetime(1970,1,1)).total_seconds()
    print "TEST SET query: %s - %s " % ((t1.day, t1.month), (t2.day, t2.month))
    query = {"start_time": {"$gt": start_seconds, "$lte": end_seconds}}
    res = db.find(query)
    print "  query returned: %s episodes." % (res.count())
    list_of_test_uuids = []
    for i in res:
        list_of_test_uuids.append(i['uuid'])

    # *******************************************************************#
    #                  Obtain Episodes in ROI                            #
    # *******************************************************************#
    rospy.loginfo("0. Running ROI query from message_store")

    for roi in gs.roi_ids(soma_map, soma_config):
        str_roi = "roi_%s" % roi
        if roi != '1': continue

        print '\nROI: ', gs.type_of_roi(roi, soma_map, soma_config), roi

        query = {"soma_roi_id": str(roi)}
        res = db.find(query)
        print "  query returned: %s episodes in region %s." % (res.count(), roi)

        all_episodes = {}
        trajectory_times = []
        cnt=0
        for cnt, trajectory in enumerate(res):
            all_episodes[trajectory["uuid"]] = Mongodb_to_list(trajectory["episodes"])
            cnt+=1

        if len(all_episodes) < 12:
            print "Not enough episodes in region %s to learn model." % roi
            continue

        # **************************************************************#
        #            Activity Graphs/Code_book/Histograms               #
        # **************************************************************#
        rospy.loginfo('Generating Activity Graphs')

        params, tag = gh.AG_setup(input_data, date, str_roi)
        print "INFO: ", params, tag, activity_graph_dir
        print "NOTE: Currently using Object ID in the graphlets"
        """
        1. Use specific object ID.
        2. Use object type info.
        3. Encode all objects as "object".
        """
        gh.generate_graph_data(all_episodes, activity_graph_dir, params, tag, obj_type = 1)

        # **************************************************************#
        #           Generate Feature Space from Histograms              #
        # **************************************************************#
        rospy.loginfo('Generating Feature Space')

        feature_space = gh.generate_feature_space(activity_graph_dir, tag)
        (code_book, graphlet_book, X_source) = feature_space
        print "code_book length = ", len(feature_space[0])
        print "number of uuids = ", len(X_source.keys())

        test_list_for_region = set(X_source.keys()) & set(list_of_test_uuids)
        print "len of test set %s in region: %s" % (len(test_list_for_region), roi)

        # **************************************************************#
        #                    Create a similarty space                   #
        # **************************************************************#
        # rospy.loginfo('Create Similarity Space')

        # similarity_space = get_similarity_space(feature_space)
        # dictionary_of_similarity = {}

        # for i in similarity_space:
        #     key = np.sum(i)
        #     if key in dictionary_of_similarity:
        #         dictionary_of_similarity[key] += 1
        #     else:
        #         dictionary_of_similarity[key] = 1

                ## print "similarty space matches =" #Note: Reducing +ve histogram counts to 1
                ## for key, cnt in dictionary_of_similarity.items():
                ## print key, cnt

        # **************************************************************#
        #                    Learn a Clustering model                   #
        # **************************************************************#
        rospy.loginfo('Learning on Feature Space')
        params, tag = gh.AG_setup(input_data, date, str_roi)
        smartThing = Learning(f_space=feature_space, roi=str_roi, vis=True)

        rospy.loginfo('Good ol k-Means')
        smartThing.split_data_on_test_set(list_of_test_uuids)
        smartThing.kmeans()
        smartThing.kmeans_cluster_radius()

        # **************************************************************#
        #                  TEST CASES - final week                      #
        # **************************************************************#

        test_set_distances, novelty_info = smartThing.kmeans_test_set()

        # **************************************************************#
        #                     Visualise Clusters                        #
        # **************************************************************#
        if plotting:
            for cnt, (label, uuids_list) in enumerate(smartThing.show_in_rviz.items()):
                query = ot.make_query(uuids_list)
                q = ot.query_trajectories(query, True, "direction_red_green")
                if cnt+1 < len(smartThing.show_in_rviz.keys()): raw_input("\npress enter for next cluster")
                else: raw_input("\npress enter novel Test set")

            #novel_uuids
            query = ot.make_query(novelty_info[0])
            q = ot.query_trajectories(query, True, "direction_red_green")
            raw_input("\npress enter for not novels...")

            #not_novel_uuids
            query = ot.make_query(novelty_info[1])
            q = ot.query_trajectories(query, True, "direction_red_green")
            raw_input("\npress enter to exit")

        sys.exit(1)
        
        # *******************************************************************#
        #                    Temporal Analysis                               #
        # *******************************************************************#
        rospy.loginfo('Learning Temporal Measures')
        # print "traj times = ", trajectory_times, "\n"
        smartThing.time_analysis(trajectory_times, plot=plotting)
        # Future: Save a dictionary of IDs, timestamps and cluster composition for further analysis
        #smartThing.methods["temporal_list_of_uuids"] = trajectory_times

        # Add the region knowledge to smartThing - Future: make modula.
        try:
            smartThing.methods["roi_knowledge"] = roi_knowledge[roi]
            smartThing.methods["roi_temp_list"] = roi_temp_list[roi]
        except KeyError:
            smartThing.methods["roi_knowledge"] = 0
            smartThing.methods["roi_temp_list"] = [0] * 24

        # Future: create a msg type and upload everything to Mongodb
        # Future: Make learning incremental, therefore load, learn, save.
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
    rospy.init_node("offline_trajectory_learner")

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
