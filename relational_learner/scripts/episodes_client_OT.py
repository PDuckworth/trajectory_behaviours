#!/usr/bin/env python

"""Creates Episodes for Trajectory data which match the query
   trajectory_query_service needs to be running for Obtain_trajectory to run"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"


import sys
import rospy
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
    #query ='''{"uuid": {"$exists" : "True"}}'''
    #query ='''{"uuid": {"$exists" : "True"}}'''
    #query ='''{"uuid": "b74c11b7-e196-5e93-a1c0-a9fb6b93866f"}''' # 17000 poses
    #query ='''{"uuid": "3d99e112-2c92-52e7-81bf-61c3064cbb0d"}''' # 4700 poses
    query ='''{"uuid": "ff75912c-ae5a-5658-8689-d1cf8f722cde"}''' #8 episodes
    q = ot.query_trajectories(query)


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
























