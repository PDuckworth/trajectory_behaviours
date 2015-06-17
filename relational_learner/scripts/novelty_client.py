#!/usr/bin/env python

import sys
import rospy
from std_msgs.msg import String
from relational_learner.srv import *
from relational_learner.msg import *
import numpy as np
import cv2
import cv_bridge
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image
import os
import cPickle as pickle
import relational_learner.learningArea as la

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
        self.pub_T = rospy.Publisher('/novalty/temporal_novalty', Image, latch=True, queue_size=1)
        self.pub = rospy.Publisher("/trajectory_behaviours/novel_trajectory", String, queue_size=10)
        rospy.Subscriber("/trajectory_behaviours/episodes", episodes_to_novelty, self.callback)
        self.smartThing = {}
        self.pc = {}
        self.pf = {}
        print '############################################################'
        print '>>>>>PAUL<<<<<<, currently am reading only roi_2_smartThing.'
        print '############################################################'
        file_ = os.path.join('/home/lucie01/STRANDS/learning/roi_2_smartThing.p')
        self.smartThing['2'] = smartThing=la.Learning(load_from_file=file_)

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

            if len(self.ret.temporal_nov)==0: self.ret.temporal_nov=[0, 0, 0]
            
            cardcheck_pub = self.novlogic.cardcheck_msg(self.uuid, self.ret)
            if cardcheck_pub != "":
                self.pub.publish(self.uuid)
                self.published_uuids.append(self.uuid)

            if self.vis: 
                self.novlogic.plot_results(self.uuid, self.rotate)
                self.call_temporal_graph()


    def call_temporal_graph(self):
        print "Region = ", self.roi
        (start_time, pc, pf) = self.ret.temporal_nov

        if self.roi not in self.pc:
            self.pc[self.roi],self.pf[self.roi] = la.Learning.temporal_plot(self.smartThing[self.roi])
        #print self.pc,pc
        #print self.pf,pf
        print start_time
        print '----------------------------'
        """

       
        check /src/relational_learning/learningArea.py and plot_temporal for details.
        """


class NoveltyScoreLogic(object):
    def __init__(self):
        self.uuid = ""
        self.spatial_results = {}
        self.keep_ids = []
        self.pub_S = rospy.Publisher('/novalty/spatial_novalty', Image, latch=True, queue_size=1)

    def cardcheck_msg(self, uuid, ret):
        """Tests whether UUID is novel or not"""

        threshold = 0.05 #probability of sample belonging to temporal model
        if ret.spatial_nov == []: (self.dst, self.mean, self.std) = 0, 0, 0
        else: (self.dst, self.mean, self.std) = ret.spatial_nov

        if self.dst > self.mean+self.std: spatial_nov = 1
        elif self.dst > self.mean + 2*self.std: spatial_nov = 2
        elif self.dst > self.mean + 3*self.std: spatial_nov = 3
        else: spatial_nov = 0

        if ret.temporal_nov != []:
            (start_time, temp1, temp2) = ret.temporal_nov
        else:
            temp1 = temp2 = 0

        description=""
        if spatial_nov > 0: 
            print "novelty = %s" % spatial_nov
            description = ">>> spatial novelty %s  " % spatial_nov
        else:
            print "not novel"

        #region knowledge must be > 1 minute
        if ret.roi_knowledge > 60: 
            if temp1 < threshold: description = description + ">>> temporal novelty %s"  % temp1
            if temp2 < threshold: description = description + ">>> temporal novelty %s" % temp2

        return description


    def plot_results(self, uuid, rotate):

        self.keep_ids.append(uuid)
        if uuid in self.spatial_results: self.spatial_results[uuid][0:9] = self.spatial_results[uuid][1:10]
        else: self.spatial_results[uuid] = [0]*10
        self.spatial_results[uuid][9] = self.dst 

        #print "keeps", self.keep_ids
        remove = []
        if rotate:
            for key in self.spatial_results:
                if key not in self.keep_ids:    remove.append(key)
            self.keep_ids = []

            for i in remove:
                del self.spatial_results[i]
            #print "remove", remove
            self.call_spatial_graph() #Load the graph only if all people in the scene have been analysed


    def call_spatial_graph(self):
        #print "PLOTTING DICT: ", self.spatial_results.keys()
       
        fig = plt.figure()
        fig.clf()
        ax = fig.add_subplot(111)
        y_max=0
        for cnt, (key,values) in enumerate(self.spatial_results.items()):
            x=[]
            y=[]
            for x_, y_ in enumerate(values):
                if y_ == 0: continue 
                x.append(x_)
                y.append(y_)
            print "X, Y = ", x, y
            if max(y)>y_max: y_max=max(y) 
            plt.plot(x,y, '+--', linewidth=1.0, label=key)

        ax.legend()
        plt.xlim(0, 10)
        plt.ylim(0, 2*y_max)

        ax.set_ylabel("Distance to Learnt Clusters")
        ax.set_xlabel("time (in trajectory_publisher msgs)")
        #ax.set_yticklabels(regions)
        plt.savefig('/tmp/nov_gr.png', bbox_inches='tight', dpi=100)
        img = cv2.imread('/tmp/nov_gr.png')
        msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.pub_S.publish(msg)





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
    frame = 1
    img = np.zeros((10,10,3),dtype=np.uint8)
    cv2.imwrite('/tmp/act_gr.png',img)
    cv2.imwrite('/tmp/nov_gr.png',img)
    
    
    pub_A = rospy.Publisher('/novalty/activity_graph', Image, latch=True, queue_size=1)
    while not rospy.is_shutdown():
        if vis_graph:
            try:
                #img = cv2.imread('/tmp/act_gr.png')
                msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
                pub_A.publish(msg)
            except Exception:
                pass

    """
    rospy.spin()
    """
