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


def query_episodes_using_dates(start, end, db):
    start_seconds = (start-datetime.datetime(1970,1,1)).total_seconds()
    end_seconds = (end-datetime.datetime(1970,1,1)).total_seconds()
    print "\nDate query: %s - %s " % ((start.day, start.month), (end.day, end.month))
    query = {"start_time": {"$gt": start_seconds, "$lte": end_seconds}}
    res = db.find(query)
    print "  query returned: %s episodes." % (res.count())
    list_of_uuids = [ i['uuid'] for i in res]
    return list_of_uuids

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
    db = client['message_store'][episode_store]
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
    #                 Get Multiple Train/Test Sets                       #
    # *******************************************************************#
    #Use the final week of data as a test set
    t1 = datetime.datetime(2015, 4, 26)
    t2 = datetime.datetime(2015, 5, 9)
    week0_uuids = query_episodes_using_dates(t1,t2, db)

    t1 = datetime.datetime(2015, 5, 9)
    t2 = datetime.datetime(2015, 5, 16)
    week1_uuids = query_episodes_using_dates(t1,t2, db)

    t1 = datetime.datetime(2015, 5, 16)
    t2 = datetime.datetime(2015, 5, 23)
    week2_uuids = query_episodes_using_dates(t1,t2, db)

    t1 = datetime.datetime(2015, 5, 23)
    t2 = datetime.datetime(2015, 5, 30)
    week3_uuids = query_episodes_using_dates(t1,t2, db)

    t1 = datetime.datetime(2015, 5, 30)
    t2 = datetime.datetime(2015, 6, 6)
    week4_uuids = query_episodes_using_dates(t1,t2, db)

    t1 = datetime.datetime(2015, 6, 6)
    t2 = datetime.datetime(2015, 6, 13)
    week5_uuids = query_episodes_using_dates(t1,t2, db)

    all = len(week0_uuids)+len(week1_uuids)+len(week2_uuids)+len(week3_uuids)+\
        len(week4_uuids)+len(week5_uuids)
    print "%s data queried from %s" % (all, episode_store)


    # *******************************************************************#
    #                  Obtain Episodes in ROI                            #
    # *******************************************************************#
    rospy.loginfo("0. Running ROI query from message_store")

    region_data_by_week = {}
    for roi in gs.roi_ids(soma_map, soma_config):
        str_roi = "roi_%s" % roi
        if roi != '1': continue
        region_data_by_week[roi] = {"wk0":[],"wk1":[],"wk2":[],"wk3":[],"wk4":[],"wk5":[]}

        print '\nROI: ', gs.type_of_roi(roi, soma_map, soma_config), roi

        query = {"soma_roi_id": str(roi)}
        res = db.find(query)
        print "  query returned: %s episodes in region %s." % (res.count(), roi)

        for i in res:
            if i['uuid'] in week0_uuids: region_data_by_week[roi]["wk0"].append(i['uuid'])
            elif i['uuid'] in week1_uuids: region_data_by_week[roi]["wk1"].append(i['uuid'])
            elif i['uuid'] in week2_uuids: region_data_by_week[roi]["wk2"].append(i['uuid'])
            elif i['uuid'] in week3_uuids: region_data_by_week[roi]["wk3"].append(i['uuid'])
            elif i['uuid'] in week4_uuids: region_data_by_week[roi]["wk4"].append(i['uuid'])
            elif i['uuid'] in week5_uuids: region_data_by_week[roi]["wk5"].append(i['uuid'])

        #for k,v in region_data_by_week[roi].items(): print k, len(v)
        r = region_data_by_week[roi]
        #keep_sample = set(r["wk1"])^ set(r["wk2"])^ set(r["wk3"])^ set(r["wk4"]) ^ set(r["wk5"])
        keep_sample = set(r["wk1"]) ^ set(r["wk5"])
        print "Keeping sample from Week 1 and 5: %s" % len(keep_sample)

        all_episodes = {}
        trajectory_times = []
        cnt=0
        res = db.find(query)
        for cnt, trajectory in enumerate(res):
            if trajectory["uuid"] not in keep_sample: continue

            all_episodes[trajectory["uuid"]] = Mongodb_to_list(trajectory["episodes"])
            cnt+=1

        if len(all_episodes) < 12:
            print "Not enough episodes in region %s to learn model." % roi
            continue
        else: print "%s episodes in region %s." % (len(all_episodes), roi)

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

        test_list_for_region = set(X_source.keys()) & set(week5_uuids)
        print "len of test set %s in region: %s" % (len(test_list_for_region), roi)

        # **************************************************************#
        #                    Learn a Clustering model                   #
        # **************************************************************#
        rospy.loginfo('Learning...')
        params, tag = gh.AG_setup(input_data, date, str_roi)
        smartThing = Learning(f_space=feature_space, roi=str(roi), vis=plotting)

        smartThing.split_data_on_test_set(scale=True, test_set=test_list_for_region)

        # **************************************************************#
        #                           Learn KMEANS                        #
        # **************************************************************#
        rospy.loginfo('Good ol k-Means')
        smartThing.kmeans()
        smartThing.cluster_radius(method="kmeans")
        test_set_distances, novelty_info = smartThing.kmeans_test_set()
        if plotting: visualise_clusters(smartThing, novelty_info)

        # **************************************************************#
        #                 Learn a AffinityPropagation                   #
        # **************************************************************#
        rospy.loginfo('Learning AffinityPropagation')
        smartThing.tf_idf_cosine_similarity_matrix()
        smartThing.AffinityPropagation()
        smartThing.cluster_radius(method="affinityprop")
        if plotting: visualise_clusters(smartThing)


        # **************************************************************#
        #                    Upload model to MongoDB                    #
        # **************************************************************#
        #smartThing.save(mongodb=True, msg_store="spatial_qsr_models_offline")
        #smartThing.save(learning_area)  #save to file
        print "Learnt models for: "
        for key in smartThing.methods:
            print "    ", key

        #something = Learning(load_from_file='mongodb', roi=roi)

    print "COMPLETED LEARNING PHASE"
    return


def visualise_clusters(smartThing, novelty_info=[]):
    N = max(smartThing.show_in_rviz.keys())
    print "visualising first cluster: 0"
    for cnt, (label, uuids_list) in enumerate(smartThing.show_in_rviz.items()):
        query = ot.make_query(uuids_list)
        q = ot.query_trajectories(query, True, "direction_red_green")
        if cnt < N: raw_input("\npress enter for next cluster: %i" % (cnt+1))

    if len(novelty_info)>1 and cnt == N:
        raw_input("\npress enter for novel trajectories")
        #novel_uuids
        query = ot.make_query(novelty_info[0])
        q = ot.query_trajectories(query, True, "direction_red_green")
        raw_input("\npress enter for not novels...")

        #not_novel_uuids
        query = ot.make_query(novelty_info[1])
        q = ot.query_trajectories(query, True, "direction_red_green")
        raw_input("\npress enter to exit method")


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
