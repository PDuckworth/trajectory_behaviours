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

from novelTrajectories.traj_data_reader import *
import novelTrajectories.config_utils as util
import relational_learner.obtain_trajectories as ot
import relational_learner.trajectory_analysis as th
import relational_learner.graphs_handler as gh
from relational_learner.learningArea import *
from split_trajectory_test_set import *

from offline_kmeans_testing import *


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

def run_all(plotting, episode_store, learn_methods, qsr_type, publishers):

	# *******************************************************************#
	#               Set up Directories and msg stores                    #
	# *******************************************************************#
	methods = [i for i in learn_methods.split(",")]

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
	print "\n%s data queried from %s" % (all, episode_store)

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

		query = {"soma_roi_id": str(roi),
				 "qsr_type" : qsr_type}
		print query
		res = db.find(query)
		print "  query returned: %s episodes in region %s." % (res.count(), roi)

		for i in res:
			if i['uuid'] in week0_uuids: region_data_by_week[roi]["wk0"].append(i['uuid'])
			elif i['uuid'] in week1_uuids: region_data_by_week[roi]["wk1"].append(i['uuid'])
			elif i['uuid'] in week2_uuids: region_data_by_week[roi]["wk2"].append(i['uuid'])
			elif i['uuid'] in week3_uuids: region_data_by_week[roi]["wk3"].append(i['uuid'])
			elif i['uuid'] in week4_uuids: region_data_by_week[roi]["wk4"].append(i['uuid'])
			elif i['uuid'] in week5_uuids: region_data_by_week[roi]["wk5"].append(i['uuid'])


		##"""SAVE DATA FOR LATER"""##
		#for k,v in region_data_by_week[roi].items(): print k, len(v)
		r = region_data_by_week[roi]

		keep_samples = set(r["wk0"]) ^ set(r["wk5"])

		#keep_samples = set(r["wk0"])^set(r["wk1"])^ set(r["wk2"])^ set(r["wk3"])^ set(r["wk4"]) ^ set(r["wk5"])

		#dict_samples = {"week0":set(r["wk0"]), "week1":set(r["wk1"]), "week2":set(r["wk2"]),
		#    "week3":set(r["wk3"]),"week4":set(r["wk4"]), "week5":set(r["wk5"])}
		##To create the split-trajectories (by seq) used for the trajectory predictions
		#pickle.dump(list(keep_samples), open('/home/strands/STRANDS/TESTING/roi_1_all_uuids.p', "w"))
		##pickle.dump(dict_samples, open('/home/strands/STRANDS/TESTING/roi_1_dict_uuids.p', "w"))

		print "Cross Validation: keep all 6 weeks = %s" % len(keep_samples)
		print "  week 0 = %s, %s" % (len(week0_uuids), len(r["wk0"]))
		print "  week 1 = %s, %s" % (len(week1_uuids), len(r["wk1"]))
		print "  week 2 = %s, %s" % (len(week2_uuids), len(r["wk2"]))
		print "  week 3 = %s, %s" % (len(week3_uuids), len(r["wk3"]))
		print "  week 4 = %s, %s" % (len(week4_uuids), len(r["wk4"]))
		print "  week 5 = %s, %s" % (len(week5_uuids), len(r["wk5"]))

		all_episodes = {}
		trajectory_times = []
		cnt=0
		res = db.find(query)
		for cnt, trajectory in enumerate(res):
			if trajectory["uuid"] not in keep_samples: continue

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

		feature_space = gh.generate_feature_space(activity_graph_dir, tag, __out = False)
		(code_book_hashes, code_book, X_source) = feature_space

		print "code_book length = ", len(code_book_hashes)
		print "total number of datapoints = ", len(X_source.keys())

		# **************************************************************#
		#                       CROSS VALIDATION                        #
		# **************************************************************#
		#cv_weeks = [0,1,2,3,4,5]
		cv_weeks = [5]

		cluster_scores, traj_scores = {}, {}
		uuid_weeks = [week0_uuids, week1_uuids, week2_uuids, week3_uuids, week4_uuids, week5_uuids]

		for test_week in cv_weeks:

			cluster_scores[test_week] = {}
			traj_scores[test_week] = {}

			print "CV week %s is test week" % test_week
			region_test_set = list( set(X_source.keys()) & set(uuid_weeks[test_week]))

			region_train_set = []
			for i in X_source.keys():
				if "_test_seq_" in i: continue #Only use the full trajectories (without _test_seq_) to train Kmeans
				if i not in uuid_weeks[test_week]: region_train_set.append(i)

			X_source_cv = {}
			for uuid in region_train_set:
				X_source_cv[uuid] = X_source[uuid]
			for uuid in region_test_set:
				X_source_cv[uuid] = X_source[uuid]

			print "  size of train set %s in region: %s" % (len(region_train_set), roi)
			print "  size of test set %s in region: %s" % (len(region_test_set), roi)
			print "  size of feature space %s in region: %s" % (len(X_source_cv.keys()), roi)

			repeats = {}    #Added for aaai analysis
			for rep in xrange(1):
				cluster_scores[test_week][rep] = {}

				# **************************************************************#
				#                    Learn a Clustering model                   #
				# **************************************************************#
				rospy.loginfo('Learning...')
				params, tag = gh.AG_setup(input_data, date, str_roi)

				feature_space = (code_book_hashes, code_book, X_source_cv)
				smartThing = Learning(f_space=feature_space, roi=str(roi), vis=plotting)
				smartThing.split_data_on_test_set(scale=True, test_set=region_test_set, feat_select=None)
				#pca, scores = smartThing.pca_investigate_variables(apply = True)

				# **************************************************************#
				#                           Learn KMEANS                        #
				# **************************************************************#
				rospy.loginfo('Good ol k-Means')
				smartThing.kmeans()
				smartThing.cluster_radius(method="kmeans")
				#smartThing.save(learning_area)

				#plotting=False
				print plotting
				print type(plotting)

				if plotting: vis_clusters(smartThing)

				#Generate the occupancy grid using the QSRs and cluster centres.
				vis=True
				theta_grids = get_cluster_occu_grids(smartThing, publishers, vis)

				"""Two scores:
				#2. How good is the prediction on partial trajectories. Does seq match seq_n's prediction.
				#1. How good is the prediction of future trajectories. This is the occupancygrid scores summed for future.
				"""

				#Score 2
				cluster_scores[test_week][rep] = test_trajectories_against_occu_grids(smartThing, theta_grids, vis)
				print "finished test week: ", repr(test_week), ". Repeat: ", repr(rep)

				#Score 1
				#traj_scores[test_week] = k_means_test_sequences(smartThing, theta_grids, vis)
				#file_wk = '/home/strands/STRANDS/TESTING/traj_seq_scores_comb_qsrs_wk_' + repr(test_week) + '.p'
				#pickle.dump(traj_scores[test_week], open(file_wk, "w"))

			file = '/home/strands/STRANDS/TESTING/theta_comp_scores_repeated.p'
			pickle.dump(cluster_scores, open(file, "w"))

		#file = '/home/strands/STRANDS/TESTING/traj_seq_scores_comb_qsrs_wk_.p'
		#pickle.dump(traj_scores, open(file, "w"))
		rospy.loginfo("Finished generating scores (and dumped them in TESTING/")

	print "COMPLETED LEARNING PHASE"
	return


def vis_clusters(smartThing, novelty_info=[]):


	numeric_clusters = [int(i) for i in smartThing.cluster_trajs.keys() ]

	N = max(numeric_clusters)
	print "visualising first cluster: 0/%s" % N
	for cnt, (label, uuids_list) in enumerate(smartThing.cluster_trajs_filtered.items()):
		query = ot.make_query(uuids_list)
		q = ot.query_trajectories(query, True, "direction_red_green")
		if cnt < N: raw_input("\npress enter for next cluster: %i / %s" % (cnt+1, N))

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
	def learn(self, plotting, episode_store, learn_methods, qsr_type, publishers):
		r = run_all(plotting, episode_store, learn_methods, qsr_type, publishers)

if __name__ == "__main__":
	parser = argparse.ArgumentParser(description="Unsupervised Learning on relational qsr epiosdes")
	parser.add_argument("-p", "--plotting", help="plotting", default=False)
	parser.add_argument("-e", "--episode_store", help="enter a mongo store of episodes", default='relational_episodes')
	parser.add_argument("-l", "--learn_methods", help="enter a learning method. Split using comma if more than one requested.", default='kmeans')
	parser.add_argument("-q", "--qsr_type", help="enter a qsr type to query the database", default='qtcb')
	args = parser.parse_args()

	print "plotting = ", bool(args.plotting)
	print "message store = ", args.episode_store
	print "learning methods = ", args.learn_methods
	print "qsr type = ", args.qsr_type

	pub_g = rospy.Publisher('/trajectory_behaviours/grid', GridCells, latch=True, queue_size=0)
	pub_o = rospy.Publisher('/trajectory_behaviours/occu', OccupancyGrid, latch=True, queue_size=0)
	publishers = (pub_g, pub_o)
	rospy.init_node("offline_trajectory_learner")

	o = Offline_Learning()
	o.learn(bool(args.plotting), args.episode_store, args.learn_methods, args.qsr_type, publishers)
