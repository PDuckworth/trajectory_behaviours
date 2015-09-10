#!/usr/bin/env python

"""Queries people_trajectory message store and uploads Episodes."""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"

import rospy
import os, sys
import pymongo
import cPickle as pickle
import datetime, time
import numpy as np
from sklearn import mixture
#import matplotlib
#matplotlib.use("Agg")
import matplotlib.pyplot as plt
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from relational_learner.srv import *
import relational_learner.obtain_trajectories as ot
from human_trajectory.trajectories import OfflineTrajectories, Trajectories
from human_trajectory.trajectory import Trajectory

from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy

def create_trajectory(log):

    t = Trajectory(str(log['uuid']))
    t.length = [0.0 for i in range(len(log['robot']))]
    t.length[-1] = log['trajectory_length']
    t.sequence_id = log['sequence_id']

    robot_pose = [
        Pose(
            Point(i['position']['x'],
                  i['position']['y'],
                  i['position']['z']),
            Quaternion(i['orientation']['x'],
                       i['orientation']['y'],
                       i['orientation']['z'],
                       i['orientation']['w']))
        for i in log['robot']
    ]
    human_pose = [
        PoseStamped(
            Header(i['header']['seq'],
                   rospy.Time(i['header']['stamp']['secs'],
                              i['header']['stamp']['nsecs']),
                   i['header']['frame_id']),
            Pose(
                Point(i['pose']['position']['x'],
                      i['pose']['position']['y'],
                      i['pose']['position']['z']),
                Quaternion(i['pose']['orientation']['x'],
                           i['pose']['orientation']['y'],
                           i['pose']['orientation']['z'],
                           i['pose']['orientation']['w'])))
        for i in log['trajectory']
    ]
    t.trajectory_displacement = log['trajectory_displacement']
    t.displacement_pose_ratio = log['displacement_pose_ratio']
    t.humrobpose = zip(human_pose, robot_pose)
    return t


class OfflineEpisodes(object):
    def __init__(self, vis=False, msg_store='relational_episodes'):

        self.client = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        )
        self.vis=vis
        self.request = 1.0
        self.msg_store = msg_store
        self.timestamps = {}
        self.uuids=[]

    def learn_threshold_from_first_days(self, t1):

        first_day_query = {"_meta.inserted_at": {"$lte": t1}}
        print "learning a good displacement/pose ratio..."
        self._retrieve_trajectories(first_day_query)

        X = np.array([])
        for log in self.trajs:
            X = np.append(X, log['displacement_pose_ratio'])

        N = np.arange(1, 11)
        models = [None for i in range(len(N))]
        for i in range(len(N)):
            models[i] = mixture.GMM(N[i]).fit(X)
        # compute the AIC and the BIC
        AIC = [m.aic(X) for m in models]
        BIC = [m.bic(X) for m in models]
        fig = plt.figure(figsize=(5, 1.7))
        fig.subplots_adjust(left=0.12, right=0.97,
                            bottom=0.21, top=0.9, wspace=0.5)

        # plot 1: data + best-fit mixture
        ax = fig.add_subplot(121)
        M_best = models[np.argmin(AIC)]

        x = np.linspace(0, max(X), 100)
        logprob, responsibilities = M_best.score_samples(x)
        pdf = np.exp(logprob)
        pdf_individual = responsibilities * pdf[:, np.newaxis]

        ax.hist(X, 30, normed=True, histtype='stepfilled', alpha=0.4)
        ax.plot(x, pdf, '-k')
        ax.plot(x, pdf_individual, '--k')
        ax.text(0.04, 0.96, "Best-fit Mixture",
                ha='left', va='top', transform=ax.transAxes)
        ax.set_xlabel('$x$')
        ax.set_ylabel('$p(x)$')

        # plot 2: AIC and BIC
        ax = fig.add_subplot(122)
        ax.plot(N, AIC, '-k', label='AIC')
        ax.plot(N, BIC, '--k', label='BIC')
        ax.set_xlabel('n. components')
        ax.set_ylabel('information criterion')
        ax.legend(loc=2)
        print M_best.means_

        self.filter_threshold = np.round(M_best.means_, 3)[0][0]
        print "learnt threshold = %s" % self.filter_threshold
        if self.vis: plt.show()
        #Maybe find the std of the points belonging to each cluster?
        #print np.mean(X), np.std(X), (np.mean(X)+np.std(X))


    # retrieve trajectories from mongodb
    def _retrieve_trajectories(self, query=None):
        #print "querying: ", query

        #Try sorting and filtering on both displacement and the ratio. remove <1m for example
        db = self.client.message_store.people_trajectory
        self.trajs = db.find(query).sort('displacement_pose_ratio', pymongo.DESCENDING)
        #self.trajs = db.find(query).sort('trajectory_displacement', pymongo.DESCENDING)
        print "number of trajectories in query: %s" % self.trajs.count()


    def _construct_from_people_trajectory(self):
        top_pc = self.trajs.count() * self.request
        logs = self.trajs[:int(top_pc)]

        for cnt, log in enumerate(logs):
            #if cnt % 100 == 0:
            #    print cnt, log['displacement_pose_ratio'], \
            #             log['trajectory_displacement']

            if self.vis: print log['displacement_pose_ratio'], self.filter_threshold
            test1 = log['displacement_pose_ratio'] < self.filter_threshold
            test2 = log['trajectory_displacement'] < 1.0 #F1 distance threshold
            #test2 = log['trajectory_displacement'] < 0.4 #F2 distance threshold

            if test1 or test2:  #learnt ratio threshold
                self.removed_ids.append(log['uuid'])
                continue

            t = create_trajectory(log)
            ret = self._episode_client(log['uuid'], \
                                      t.get_trajectory_message())

    def _episode_client(self, uuid, Trajectory):
        self.uuids.append(uuid)
        rospy.wait_for_service('/episode_service')
        proxy = rospy.ServiceProxy('/episode_service', EpisodeService)
        req = EpisodeServiceRequest(Trajectory, False, [uuid], self.msg_store)
        ret = proxy(req)
        return ret

    def query_and_visualise(self):
        print "accepted:", len(self.uuids)
        print "removed:", len(self.removed_ids)
        if len(self.uuids) != 0:
            print "visualise %s acceptable trajectories" % len(self.uuids)
            ot.query_trajectories(ot.make_query(self.uuids),\
                        True, "direction_red_green")
            raw_input("Pause...")

        if len(self.removed_ids) != 0:
            print "visualise %s rejected trajectories" % len(self.removed_ids)
            ot.query_trajectories(ot.make_query(self.removed_ids),\
                    True, "direction_red_green")
            raw_input("Pause...")


if __name__ == "__main__":
    rospy.init_node('episode_updater')

    st_0 = time.time()
    vis=False
    episode_store = 'episodes_f1_multi_new'
    oe = OfflineEpisodes(vis, episode_store)

    resutls_of_filtering = []
    all_dates = []

    #start with this range of dates
    t1_st = datetime.datetime(2015, 4, 29, 00, 0, 0, 000000)
    t2_st = datetime.datetime(2015, 4, 30, 00, 0, 0, 000000)

    # Loop using number of hours, and number of days.
    hrs = 24
    days = 46

    # Exclude these dates also:
    list_of_bank_holidays = [datetime.datetime(2015, 5, 4), \
                            datetime.datetime(2015, 5, 25)]

    # Already learnt GMM with 1 mean from day 1 data :)
    #oe.learn_threshold_from_first_days(t2_st)

    loops = days*(24/float(hrs))
    loops = 1

    for i in range(int(loops)):

        #Don't keep accumulating the kept/rejected ids - used for vis
        oe.uuids = []
        oe.removed_ids = []

        t1 = t1_st + datetime.timedelta(hours=hrs*i)
        t2 = t2_st + datetime.timedelta(hours=hrs*i)
        date = "(%s <= t < %s)" % ( (t1.day, t1.month), (t2.day, t2.month))
        all_dates.append(date)
        print "\n>> Dates queried: %s " % date
        if t1.weekday() in [5,6] or t1 in list_of_bank_holidays:
            print ">weekend / bank holiday"
            continue

        #query_start_seconds = (t1-datetime.datetime(1970,1,1)).total_seconds()
        #query_end_seconds = (t2-datetime.datetime(1970,1,1)).total_seconds()
        date_query = {"_meta.inserted_at": {"$gte": t1, "$lt": t2}}
        st = time.time()

        #Already learnt GMM with 1 mean from day 1 data :)
        oe.filter_threshold =  0.01 #F1
        #oe.filter_threshold = 0.005 #F2

        oe._retrieve_trajectories(date_query)
        oe._construct_from_people_trajectory()

        if oe.vis: oe.query_and_visualise()
        else: print "Number of accepted trajs = %s" % len(oe.uuids)

        resutls_of_filtering.append((oe.trajs.count(), len(oe.uuids), len(oe.removed_ids)))
        if len(oe.uuids) != 0: rospy.loginfo("Episodes uploaded to %s" % episode_store)

    print "TOTAL TIME TAKEN = ", time.time() - st_0

    #Print info from Trajectory Filtering
    print "date_query:                 returned:    accpeted     rejected"
    print "---"*25
    for d, v in zip(all_dates, resutls_of_filtering):
        print "%s         %s           %s          %s" % (d, v[0], v[1], v[2])
