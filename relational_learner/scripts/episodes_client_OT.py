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
        self.uuid = ''
        #self.pose = None
        self.vis = qsr_visualisation

        print "1", self.vis
        self.pub = rospy.Publisher("/trajectory_behaviours/episodes", episodesMsg, queue_size=10)
        rospy.Subscriber("/human_trajectories/trajectories/batch", Trajectories, self.callback)

    def episode_client(self, Trajectory):
        rospy.wait_for_service('/episode_service')
        proxy = rospy.ServiceProxy('/episode_service', EpisodeService)  
        req = EpisodeServiceRequest(Trajectory, self.vis)
        ret = proxy(req)
        return ret

    def callback(self, msg):
        if len(msg.trajectories) > 0:
            self.uuid = msg.trajectories[0].uuid
            #self.pose = msg.trajectories[0].trajectory[-1].pose
            self.ret = self.novelty_client(msg.trajectories[0])


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

    raw_input("finished query. Enter to continue.")

    for cnt, i in enumerate(q.res.trajectories.trajectories):

        print "\n", cnt, i.uuid

        ret = ec.episode_client(i)
        #print ret.header
        print ret.uuid, ret.soma_roi_id
        print "len = ", len(ret.episodes)
        print ret.episodes
        #ec.pub.publish(ret)
        #rospy.sleep(1) 

    #rospy.spin()
























