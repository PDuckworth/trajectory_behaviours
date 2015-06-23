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

    def __init__(self, qsr_visualisation):
        self.ret = None
        self.current_uuids_detected = []
        self.vis = qsr_visualisation

        self.pub = rospy.Publisher("/trajectory_behaviours/episodes", episodesMsg, queue_size=10)

    def episode_client(self, Trajectory):
        rospy.wait_for_service('/episode_service')
        proxy = rospy.ServiceProxy('/episode_service', EpisodeService)  
        req = EpisodeServiceRequest(Trajectory, self.vis, self.current_uuids_detected)
        ret = proxy(req)
        return ret


if __name__ == "__main__":
    rospy.init_node('episodes_client')

    if len(sys.argv) == 2:
        vis=bool(sys.argv[1])
    else:
        vis=False
    print "Usage: QSR visualisation in RVIZ not selected. Turn_on = 1. 0 by default."
    print "qsr_viz = ", vis

    ec = EpisodeClient(vis)
    query ='''{"uuid": {"$exists" : "True"}}'''

    data_dir = '/home/strands/STRANDS/'
    list_of_uuids = pickle.load(open(data_dir + 'trajectory_dump/best_uuids_filtered_by_displacement.p', "r"))
    print list_of_uuids
    query = ot.make_query(list_of_uuids)
    q = ot.query_trajectories(query)
    q.get_poses()

    ec.current_uuids_detected = []
    for trajectory in q.res.trajectories.trajectories:
        ec.current_uuids_detected.append(trajectory.uuid)
    print "\nids = %s" % ec.current_uuids_detected
    
    for cnt, trajectory in enumerate(q.res.trajectories.trajectories):
        print "\n passing ", cnt, trajectory.uuid
        ret = ec.episode_client(trajectory)
        print "returned"

        if ret.soma_roi_id != "":
            ec.pub.publish(ret)
            print "uuid = %s Episodes uploaded to database" % trajectory.uuid
        else:
            print "uuid = %s is outside of all soma roi" % trajectory.uuid

    #rospy.spin()
























