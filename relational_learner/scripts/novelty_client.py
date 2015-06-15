#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from relational_learner.srv import *
from relational_learner.msg import *
import cv2

class NoveltyClient(object):
    
    def __init__(self, threshold=0.05, vis=False):
        self.msg = None
        self.ret = None
        self.rotate = False
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
            self.rotate = msg.rotate_image
            self.ret = self.novelty_client(msg)

            print "\n", self.cnt, self.uuid
            print self.ret
            self.cnt+=1 

            if len(self.ret.temporal_nov)==0: self.ret.temporal_nov=[0, 0]
            
            cardcheck_pub = self.novlogic.cardcheck_msg(self.uuid, self.ret)
            if cardcheck_pub != "":
                self.pub.publish(self.uuid)
                print "MSG to Card Reader =", msg
                self.published_uuids.append(self.uuid)

            if self.vis: self.novlogic.plot_results(self.uuid, self.rotate)


class NoveltyScoreLogic(object):
    def __init__(self):
        self.uuid = ""
        self.spatial_results = {}

    def cardcheck_msg(self, uuid, ret):
        """Tests whether UUID is novel or not"""

        threshold = 0.05 #probability of sample belonging to temporal model
        (self.dst, self.mean, self.std) = ret.spatial_nov

        if self.dst > self.mean+self.std: spatial_nov = 1
        elif self.dst > self.mean + 2*self.std: spatial_nov = 2
        elif self.dst > self.mean + 3*self.std: spatial_nov = 3
        else: spatial_nov = 0

        if ret.temporal_nov != []:
            (temp1, temp2) = ret.temporal_nov
        else:
            temp1 = temp2 = 0

        description=""
        if spatial_nov > 0: 
            print "novelty = %s" % spatial_nov
            description = ">>> spatial novelty %s  " % spatial_novelty
        else:
            print "not novel"

        #region knowledge must be > 1 minute
        if ret.roi_knowledge > 60: 
            if temp1 < threshold: description = description + ">>> temporal novelty %s"  % temp1
            if temp2 < threshold: description = description + ">>> temporal novelty %s" % temp2

        return description


    def plot_results(self, uuid, rotate):
        
        if uuid in self.spatial_results: self.spatial_results[uuid].append(self.dst)
        else: self.spatial_results[uuid] = [self.dst]
        print "PLOTTING DICT: ", self.spatial_results
        print "ROTATE = ", rotate
        call_graph(rotate)


def call_graph(rotate):
    print "plot this graph"
    return
    

if __name__ == "__main__":
    rospy.init_node('novelty_client')
    print "novelty client running..."

    if len(sys.argv) == 2:
        vis_graph=bool(sys.argv[1])
    else:
        vis_graph=False
    print "Usage: Visualising Activity Graphs is not selected. Turn_on = 1. 0 by default."
    print "Graph viz = ", vis_graph

    nc = NoveltyClient(0.05, vis_graph)

    """
    while not rospy.is_shutdown() and vis_graph:
        try:
            img = cv2.imread('/tmp/act_gr.png')
            img = cv2.resize(img, (0,0), fx=0.5, fy=0.5)
            cv2.imshow('Activity_Graph',img)
            cv2.waitKey(1)
        except:
            pass
    """
    rospy.spin()

