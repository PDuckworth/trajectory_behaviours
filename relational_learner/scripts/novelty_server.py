#!/usr/bin/env python
import rospy
import sys, os, getpass, time
import ConfigParser
import itertools
import cPickle as pickle
from datetime import datetime
from scipy.spatial import distance

from relational_learner.msg import *
from relational_learner.srv import *

import novelTrajectories.config_utils as util
import relational_learner.graphs_handler as gh
import relational_learner.learningArea as la
from time_analysis.cyclic_processes import *
from relational_learner.Activity_Graph import Activity_Graph
from mongodb_store.message_store import MessageStoreProxy

#from std_msgs.msg import Header
import cv2
import cv_bridge
import matplotlib.pyplot as plt
from sensor_msgs.msg import Image


class Importer(object):
    def __init__(self):
        rospy.loginfo("Connecting to mongodb...")
        self._client = pymongo.MongoClient(rospy.get_param("mongodb_host"),
                                           rospy.get_param("mongodb_port"))
        self._store_client = MessageStoreProxy(collection="relational_learner")

def episodesMsg_to_list(req):
    """Convert the EpisodesMsg into a list of episodes
       EpisodesMsg: 
            std_msgs/Header header
            string soma_roi_id
            string soma_map
            string soma_config
            int64 start_time
            relational_learner/episodeMsg[] episodes
                string obj1
                string obj1_type
                string obj2
                string obj2_type
                string spatial_relation
                int32 start_frame
                int32 end_frame
    """
    ep_list = []
    for i in req.episodes.episodes:
        ep = (i.obj1, i.obj1_type, i.obj2, i.obj2_type, \
             i.spatial_relation, i.start_frame, i.end_frame)
        ep_list.append(ep)
    all_episodes = {req.episodes.uuid : ep_list}
    return all_episodes


class novelty_class(object):
    def __init__(self):
        self.pub_A = rospy.Publisher('/novelty/activity_graph', \
                    Image, latch=True, queue_size=1)
        self.pub_T = rospy.Publisher('/novelty/temporal_novelty', \
                    Image, latch=True, queue_size=1)
        self.pub_S = rospy.Publisher('/novelty/spatial_novelty', \
                    Image, latch=True, queue_size=1)   

        self.spatial_results = {}
        self.keep_ids = []
        self.rotate = False
        self.fig1 = plt.figure()
        self.fig2 = plt.figure()

    def publish_temp_image(self, pc, pf, time_of_day, roi, threshold):
        self.fig1.clf()
        ax1 = self.fig1.add_subplot(111)
        plt.sca(ax1)

        plot_interval=900.0
        timestamps, pc_all, pf_all = self.get_temporal_values(plot_interval)
        plot_vec = [t/3600.0 for t in timestamps]
        width=0.01
        plt.plot(plot_vec,pc_all, color='r', label='dynamic clustering')
        plt.plot(plot_vec,pf_all,label='GMM fitting')
        plt.plot(time_of_day/3600.0, pc, label='dyn novelty', color='c', marker='o', markersize=15)
        plt.plot(time_of_day/3600.0, pf, label='gmm novelty', color='g', marker='o', markersize=15)
        plt.plot(time_of_day/3600.0, threshold, '--', color='b', linewidth=0.5)
        plt.xlim([0,24])
        ax1.set_xticks(np.arange(0,24))

        plt.xlabel('time of day')
        plt.ylabel('probability')
        plt.legend(loc = 'best')
        filename='/tmp/temporal_plot_%s.png' % roi
        plt.savefig(filename, bbox_inches='tight', dpi=100)  
        img = cv2.imread(filename)
        activity_graph_msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.pub_T.publish(activity_graph_msg)  

    def get_temporal_values(self, plot_interval, period=86400):
        pc = []
        pf = []
        #query model at these points to generate graph:
        timestamps = np.arange(0,period,plot_interval) 
        for v in timestamps:
            pc.append(self.dyn_cl.query_clusters(v))
            pf.append(self.fitting.query_model(v))
        return timestamps, pc, pf 
        
    def publish_AG_image(self):
        img = cv2.imread('/tmp/act_gr.png')
        activity_graph_msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.pub_A.publish(activity_graph_msg)

    def plot_results(self, uuid, (dst, mean, std)):

        self.keep_ids.append(uuid)
        if uuid in self.spatial_results: self.spatial_results[uuid][0:9] = self.spatial_results[uuid][1:10]
        else: self.spatial_results[uuid] = [(0,0,0)]*10
        self.spatial_results[uuid][9] = (dst, mean, std) 

        #print "keeps", self.keep_ids
        remove = []
        if self.rotate:
            for key in self.spatial_results:
                if key not in self.keep_ids: remove.append(key)
            self.keep_ids = []

            for i in remove:
                del self.spatial_results[i]
            #print "remove", remove
            self.call_spatial_graph() #Load the graph only if all people in the scene have been analysed


    def call_spatial_graph(self):
        #print "PLOTTING DICT: ", self.spatial_results.keys()
        self.fig2.clf()
        ax2 = self.fig2.add_subplot(111)
        plt.sca(ax2)
        y_max=0
        for cnt, (key, values) in enumerate(self.spatial_results.items()):
            x=[]
            y, mean, std = [], [], []
            for x_, (y_, mean_, std_) in enumerate(values):
                if y_ == 0: continue 
                x.append(x_)
                y.append(y_)
                mean.append(mean_)
                std.append(std_)
            print "X, Y = ", x, y, mean, std
            if max(y)>y_max: y_max=max(y)
            if max(std)>y_max: y_max=max(std)
            plt.plot(x,y, '+-', linewidth=1.0, label=key)
            #plt.plot(x,mean, '--', linewidth=0.5, label='  mean')
            plt.plot(x,std, '--', linewidth=0.5, label='  mean+std')
        ax2.legend()
        plt.xlim(0, 10)
        plt.ylim(0, y_max*1.5)

        ax2.set_ylabel("Distance to Learnt Clusters")
        ax2.set_xlabel("time (in trajectory_publisher msgs)")
        #ax2.set_yticklabels(regions)
        plt.savefig('/tmp/nov_gr.png', bbox_inches='tight', dpi=100)
        img = cv2.imread('/tmp/nov_gr.png')
        spatial_graph_msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
        self.pub_S.publish(spatial_graph_msg)


    def handle_novelty_detection(self, req):

        print "visualise Graph = ", req.visualise_graphs

        """1. Get data from EpisodesMsg"""
        t0=time.time()
        uuid = req.episodes.uuid
        print "\nUUID = ", uuid
        roi = req.episodes.soma_roi_id
        print "ROI = ", roi
        eps_soma_map = req.episodes.soma_map
        eps_soma_config = req.episodes.soma_config
        start_time = req.episodes.start_time
        all_episodes = episodesMsg_to_list(req)

        episodes_file = all_episodes.keys()[0]
        print "Length of Episodes = ", len(all_episodes[episodes_file])
     
        (directories, config_path, input_data, date) = util.get_learning_config()
        (data_dir, qsr, trajs, graphs, learning_area) = directories
        #(data_dir, config_path, params, date) = util.get_qsr_config()
        (soma_map, soma_config) = util.get_map_config(config_path)
        
        if eps_soma_map != soma_map: raise ValueError("Config file soma_map not matching published episodes")
        if eps_soma_config != soma_config: raise ValueError("Config file soma_config not matching published episodes")

        params, tag = gh.AG_setup(input_data, date, roi)

        print "params = ", params
        print "tag = ", tag

        """4. Activity Graph"""
        ta0=time.time()

        activity_graphs = gh.generate_graph_data(all_episodes, data_dir, \
                params, tag, test=True, vis=req.visualise_graphs)

        if req.visualise_graphs: self.publish_AG_image()


        #print "\n  ACTIVITY GRAPH: \n", activity_graphs[episodes_file].graph 
        ta1=time.time()
        
        """5. Load spatial model"""
        print "\n  MODELS LOADED :"
        file_ = os.path.join(data_dir + 'learning/roi_' + roi + '_smartThing.p')
        smartThing=la.Learning(load_from_file=file_)
        if smartThing.flag == False: return NoveltyDetectionResponse()

        print "code book = ", smartThing.code_book

        """6. Create Feature Vector""" 
        test_histogram = activity_graphs[episodes_file].get_histogram(smartThing.code_book)
        print "HISTOGRAM = ", test_histogram
      

        """6.5 Upload data to Mongodb"""
        """activityGraphMsg:
                std_msgs/Header header
                string uuid
                string soma_roi_id
                string soma_map
                string soma_config
                int64[] codebook
                float32[] histogram
        """
        #header = req.episodes.trajectory.header
        #meta = {"map":'uob_library'}

        #tm0 = time.time()
        #ag =  activityGraphMsg(
        #            header=header, uuid=req.episodes.uuid, roi=roi, \
        #            histogram = test_histogram, codebook = smartThing.code_book, \
        #            episodes=get_episode_msg(ep.all_episodes[episodes_file]))
       
        #query = {"uuid" : str(uuid)} 
        #p_id = Importer()._store_client.update(message=ag, message_query=query,\
        #                                       meta=meta, upsert=True)
        #tm1 = time.time()

        """7. Calculate Distance to clusters"""
        estimator = smartThing.methods['kmeans']
        closest_cluster = estimator.predict(test_histogram)
        
        print "INERTIA = ", estimator.inertia_
        #print "CLUSTER CENTERS = ", estimator.cluster_centers_

        a = test_histogram
        b = estimator.cluster_centers_[closest_cluster]
        dst = distance.euclidean(a,b)
        print "\nDISTANCE = ", dst

        mean = estimator.cluster_dist_means[closest_cluster[0]]
        std = estimator.cluster_dist_std[closest_cluster[0]]
        print "Mean & std = ",  mean, std

        if dst > mean + std: spatial_nov = 1
        elif dst > mean + 2*std: spatial_nov = 2
        elif dst > mean + 3*std: spatial_nov = 3
        else: spatial_nov = 0
        print "Spatial flag = ", spatial_nov
        self.rotate = req.episodes.rotate_image
        std_1 = mean+std
        if req.visualise_graphs: self.plot_results(uuid, (dst, mean, std_1))

        """8. Time Analysis"""
        self.fitting = smartThing.methods['time_fitting']
        self.dyn_cl = smartThing.methods['time_dyn_clst']

        time_of_day = start_time%86400
        pc = self.dyn_cl.query_clusters(time_of_day)
        pf = self.fitting.query_model(time_of_day)
        
        print "PC = ", pc
        print "PF = ", pf

        if req.visualise_graphs: self.publish_temp_image(pc, pf, time_of_day, \
                    roi, req.temp_threshold)

        """9. ROI Knowledge"""
        try:
            region_knowledge = smartThing.methods['roi_knowledge']
            temporal_knowledge = smartThing.methods['roi_temp_list']
            print "Region Knowledge Score = ", region_knowledge
            print "Hourly score = ", region_knowledge

            t = datetime.fromtimestamp(start_time)
            print "Date/Time = ", t
            th = temporal_knowledge[t.hour]
            print "Knowledge per hour = ", th

        except KeyError:
            print "No Region knowledge in `region_knowledge` db"
            th = 0
     
        print "\n Service took: ", time.time()-t0, "  secs."
        print "  AG took: ", ta1-ta0, "  secs."
        #print "  Mongo upload took: ", tm1-tm0, "  secs."

        return NoveltyDetectionResponse(spatial_nov, [pc, pf], th)


def calculate_novelty():
    rospy.init_node('novelty_server')
    n = novelty_class()
    

                        #service_name       #serive_type       #handler_function
    s = rospy.Service('/novelty_detection', NoveltyDetection, \
                    n.handle_novelty_detection)
    print "Ready to detect novelty..."
    rospy.spin()



if __name__ == "__main__":
    calculate_novelty()

