#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from relational_learner.srv import *
from relational_learner.msg import *
import numpy as np
import matplotlib.pyplot as plt

class NoveltyClient(object):
    
    def __init__(self, threshold=0.05, vis=False):
        self.msg = None
        self.ret = None

        self.uuid = ''
        self.threshold=threshold
        self.novlogic = NoveltyScoreLogic()
        self.cnt=0
        self.vis=vis
        self.published_uuids = []
        self.pub = rospy.Publisher("/trajectory_behaviours/novel_trajectory", String, queue_size=10)
        rospy.Subscriber("/trajectory_behaviours/episodes", episodes_to_novelty, self.callback)


    def novelty_client(self, msg):
        rospy.wait_for_service('/novelty_detection')
        proxy = rospy.ServiceProxy('/novelty_detection', NoveltyDetection)  
        req = NoveltyDetectionRequest(msg, self.vis)
        ret = proxy(req)
        return ret

    def callback(self, msg):

        if len(msg.uuid) > 0:
            self.uuid = msg.uuid
            self.roi = msg.soma_roi_id
            self.ret = self.novelty_client(msg)

            print "\n", self.cnt, self.uuid
            print self.ret
            self.cnt+=1 

            if len(self.ret.temporal_nov)==0: self.ret.temporal_nov=[0, 0]
            
            cardcheck_pub = self.novlogic.cardcheck_msg(self.uuid, self.ret)
            if cardcheck_pub != "":
                self.pub.publish(self.uuid)
                self.published_uuids.append(self.uuid)


class NoveltyScoreLogic(object):
    def __init__(self):
        self.uuid = ""

    def cardcheck_msg(self, uuid, ret):
        """Tests whether UUID is novel or not"""

        threshold = 0.05 #probability of sample belonging to temporal model

        if ret.temporal_nov != []:
            (temp1, temp2) = ret.temporal_nov
        else:
            temp1 = temp2 = 0

        description=""
        if ret.spatial_nov > 0: 
            print "novelty = %s" % ret.spatial_nov
            description = ">>> spatial novelty %s  " % ret.spatial_nov
        else:
            print "not novel"

        #region knowledge must be > 1 minute
        if ret.roi_knowledge > 60: 
            if temp1 < threshold: description = description + ">>> temporal novelty %s"  % temp1
            if temp2 < threshold: description = description + ">>> temporal novelty %s" % temp2

        return description


if __name__ == "__main__":
    rospy.init_node('novelty_client')
    print "novelty client running..."

    if len(sys.argv) == 2:
        vis_graph=bool(sys.argv[1])
    else:
        vis_graph=False
    print "Usage: Visualising Activity Graphs is not selected. Turn_on = 1. 0 by default."
    print "Graph viz = ", vis_graph

    nc = NoveltyClient(threshold=0.05, vis=vis_graph)
    rospy.spin()
