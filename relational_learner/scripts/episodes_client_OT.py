#!/usr/bin/env python

"""Creates Episodes for Trajectory data which match the query
   trajectory_query_service needs to be running for Obtain_trajectory to run"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"


import sys
import rospy
import cPickle as pickle
from std_msgs.msg import String
from relational_learner.srv import *
from relational_learner.msg import *
from human_trajectory.msg import Trajectories
import relational_learner.obtain_trajectories as ot

class EpisodeClient(object):

    def __init__(self, qsr_visualisation=False):
        self.ret = None
        self.current_uuids_detected = []
        self.vis = qsr_visualisation

        self.pub = rospy.Publisher("/trajectory_behaviours/episodes", episodesMsg, queue_size=10)

    def episode_client(self, Trajectory):
        rospy.wait_for_service('/episode_service')
        proxy = rospy.ServiceProxy('/episode_service', EpisodeService)  
        req = EpisodeServiceRequest(Trajectory, self.vis, self.current_uuids_detected, \
                                    "relational_episodes")
        ret = proxy(req)
        return ret


if __name__ == "__main__":
    rospy.init_node('episodes_client')

    ec = EpisodeClient()
    #query ='''{"uuid": {"$exists" : "True"}}'''

    data_dir = '/home/strands/STRANDS/'
    list_of_uuids = pickle.load(open(data_dir + 'trajectory_dump/filtered_on_disp_roi_1_end1.p', "r"))

    query = ot.make_query(list_of_uuids)
    q = ot.query_trajectories(query)
    q.get_poses()

    ec.current_uuids_detected = list_of_uuids
    
    for cnt, trajectory in enumerate(q.res.trajectories.trajectories):
        print cnt, trajectory.uuid
        ret = ec.episode_client(trajectory)

        """
        if ret.soma_roi_id != "":
            ec.pub.publish(ret)
            print "uuid = %s Episodes uploaded to database" % trajectory.uuid
        else:
            print "uuid = %s is outside of all soma roi" % trajectory.uuid
        """
























