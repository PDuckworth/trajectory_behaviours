#!/usr/bin/env python

"""Queries Episode message store and saves timestamps per region."""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"

import rospy
import os, sys
import pymongo
import cPickle as pickle
import datetime, time
import numpy as np

class Temporal_Episodes(object):
    def __init__(self):

        self.client = pymongo.MongoClient(
            rospy.get_param("mongodb_host", "localhost"),
            rospy.get_param("mongodb_port", 62345)
        )
        self.timestamps = {}

    def _retrieve_episodes(self, query=None):
        print "querying: ", query

        db = self.client.message_store.relational_episodes
        self.episodes = db.find(query).sort('soma_roi_id', pymongo.ASCENDING)
        print "number of episodes in query: %s" % self.episodes.count()


    def _construct_region_timestamps(self):

        for cnt, log in enumerate(self.episodes):
            #if cnt % 10 == 0:
            #   print cnt, log['soma_roi_id'], log['start_time']

            if log['soma_roi_id'] not in self.timestamps:
                self.timestamps[log['soma_roi_id']] = [log['start_time']]
            else:
                self.timestamps[log['soma_roi_id']].append([log['start_time']][0])


if __name__ == "__main__":
    rospy.init_node('episode_time_querier')
    # Loop over multiple dates.

    st_0 = time.time()
    te = Temporal_Episodes()

    query = {"soma_roi_id": {"$exists":"true"}}
    te._retrieve_episodes(query)
    te._construct_region_timestamps()

    for k, v in te.timestamps.items():
        print k, len(v)

    filename = '/home/strands/STRANDS//trajectory_dump/G4S/by_region_filtered_times.p'
    with open(filename, "wb") as f:
        pickle.dump(te.timestamps, f)
    print("success")

    print "TOTAL TIME TAKEN = ", time.time() - st_0
