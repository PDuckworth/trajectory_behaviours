#!/usr/bin/env python

#__author__      = "Paul Duckworth"
#__copyright__   = "Copyright 2015, University of Leeds"

import rospy
import sys, os, getpass, time
import ConfigParser
import itertools
from datetime import datetime

from soma_geospatial_store.geospatial_store import *
import relational_learner.obtain_trajectories as ot
import novelTrajectories.traj_data_reader as tdr
import novelTrajectories.config_utils as util

from relational_learner.msg import episodeMsg, episodesMsg
from relational_learner.srv import *
from mongodb_store.message_store import MessageStoreProxy

#from std_msgs.msg import Header

class stitch_uuids(object):
    def __init__(self):
        self.stored_uuids = []
        self.stored_qsrs = {}
        self.all_uuids = []

    def check_stored_uuids(self, current_uuids):
        print "checking..."
        for uuid in self.stored_uuids:
            if uuid not in current_uuids:
                self.stored_uuids.remove(uuid)
                if uuid in self.stored_qsrs: del self.stored_qsrs[uuid]


    def merge_qsr_worlds(self, uuid, data_reader):
        """Merges together trajectories which are published
           with the same UUID as something currently being published.
           Merge occurs after QSRs have been generated.
        """

        #If new UUID
        if uuid not in self.stored_uuids:
            print "NOTE: NEW UUID"
            self.stored_uuids.append(uuid)
            self.stored_qsrs[uuid] = data_reader.spatial_relations[uuid].trace
            return data_reader

        #If UUID is still being published
        else:
            print "NOTE: QSR Trace stitched"
            #print "NEW trace = ", len(data_reader.spatial_relations[uuid].trace)
            #print "STORED trace lenth = ", len(self.stored_qsrs[uuid])
            len_of_stored = len(self.stored_qsrs[uuid])

            for (key,pose) in data_reader.spatial_relations[uuid].trace.items():
                self.stored_qsrs[uuid][key+len_of_stored] = pose
            #print "STITCHED length = ", len(self.stored_qsrs[uuid])  #QSRs stitched together
            data_reader.spatial_relations[uuid].trace = self.stored_qsrs[uuid]
            return data_reader


class Importer(object):
    def __init__(self):
        rospy.loginfo("Connecting to mongodb...")
        self._client = pymongo.MongoClient(rospy.get_param("mongodb_host"),
                                           rospy.get_param("mongodb_port"))

        #self._store_client = MessageStoreProxy(collection="relational_episodes")


def get_episode_msg(all_episodes):
    episodes = []

    for key, episode_list in all_episodes.items():
        #print key
        for cnt, ep in enumerate(episode_list):
            #print cnt, ep
            msg = episodeMsg()
            msg.obj1, msg.obj1_type, msg.obj2, msg.obj2_type, \
            msg.spatial_relation, msg.start_frame, msg.end_frame = ep[0:7]

            episodes.append(msg)
    return episodes



def get_poses(trajectory_message):
    traj = []
    for entry in trajectory_message.trajectory:
        x=entry.pose.position.x
        y=entry.pose.position.y
        z=entry.pose.position.z
        traj.append((x,y,z))
    return traj



def handle_episodes(req):
    """     1. Take trajectory as input
            2. Query mongodb for the region and objects
            3. Pass to QSRLib data parser
            4. Generate Episodes
    """

    t0=time.time()

    """1. Trajectory Message"""
    uuid = req.trajectory.uuid
    start_time = req.trajectory.start_time.secs
    print "\n1. Analysing trajectory: %s" %uuid
    visualise_qsrs = req.qsr_visualisation
    print "vis qsr = ", visualise_qsrs
    current_uuids = req.current_uuids_detected

    (data_dir, config_path) = util.get_path()
    (soma_map, soma_config) = util.get_map_config(config_path)

    trajectory_poses = {uuid : get_poses(req.trajectory)}
    print "LENGTH of Trajectory: ", len(trajectory_poses[uuid])
    ti1=time.time()

    """2. Region and Objects"""
    gs = GeoSpatialStoreProxy('geospatial_store', 'soma')
    msg_store = GeoSpatialStoreProxy('message_store', 'soma')
    two_proxies = TwoProxies(gs, msg_store, soma_map, soma_config)

    roi = two_proxies.trajectory_roi(req.trajectory.uuid, trajectory_poses[uuid])
    if roi is None: return EpisodeServiceResponse(uuid=uuid)

    ti2=time.time()
    #This currently takes too long!
    objects = two_proxies.roi_objects(roi)
    ti3=time.time()

    print "\nROI: ", roi
    #print "\n  Objects: ", objects
    if objects == None:
        print "no objects in region"
        return EpisodeServiceResponse(uuid=uuid)

    """2.5 Get the closest objects to the trajectory"""
    closest_objs_to_trajs = ot.trajectory_object_dist(objects, trajectory_poses, select=5)
    ti4=time.time()

    """3. QSRLib data parser"""
    tq0=time.time()

    qsr_reader = tdr.Trajectory_Data_Reader(objects=objects, \
                                        trajectories=trajectory_poses, \
                                        objs_to_traj_map = closest_objs_to_trajs, \
                                        roi=roi, vis=visualise_qsrs, \
                                        current_uuids=current_uuids)

    #tr = qsr_reader.spatial_relations[uuid].trace
    #for i in tr:
    #    print i, tr[i].qsrs['trajectory,Plant (tall)_7'].qsr, type(tr[i].qsrs['trajectory,Plant (tall)_7'].qsr)

    params_str = qsr_reader.params_str

    """3.5 Check the uuid is new (or stitch QSRs together)"""
    print "currently in memory = ", stitching.stored_uuids
    stitching.check_stored_uuids(current_uuids)
    print "currently published ids = ", current_uuids
    print "still in memory = ", stitching.stored_uuids

    stitching.merge_qsr_worlds(uuid, qsr_reader)
    tq1=time.time()

    """4. Episodes"""
    te0=time.time()
    ep = tdr.Episodes(reader=qsr_reader)
    #print "\n  ALL EPISODES :"
    #for t in ep.all_episodes:
    #    for o in ep.all_episodes[t]:
    #        print ep.all_episodes[t][o]

    episodes_file = ep.all_episodes.keys()[0] #This gives the ID of the Trajectory
    uuid, start, end = episodes_file.split('__')  #Appends the start and end frame #
    print episodes_file
    te1=time.time()

    """6.5 Upload data to Mongodb"""

    tm0 = time.time()
    h = req.trajectory.header
    meta = {}
    msg = episodesMsg(header=h, uuid=uuid, soma_roi_id=str(roi),  soma_map=soma_map, \
                    soma_config=soma_config, start_time=start_time, \
                    qsr_type= qsr_reader.qsr, qsr_params=params_str, \
                    episodes=get_episode_msg(ep.all_episodes[episodes_file]))

    #MongoDB Query - to see whether to insert new document, or update an existing doc.
    query = {"uuid" : str(uuid),
             "qsr_type": qsr_reader.qsr}
    i = Importer()
    i._store_client = MessageStoreProxy(collection=req.message_store_name)
    p_id = i._store_client.update(message=msg, message_query=query, meta=meta, upsert=True)
    tm1 = time.time()

    print "\nService took: ", time.time()-t0, "  secs."
    print "  Initialising srv message: ", ti1-t0, "  secs."
    print "  Initialising proxies: ", ti2-ti1, "  secs."
    print "  Initialising objects: ", ti3-ti2, "  secs."
    print "  Initialising object distances: ", ti4-ti3, "  secs."
    print "  Data Reader took: ", tq1-tq0, "  secs."
    print "  Episodes took: ", te1-te0, "  secs."
    print "  Mongo upload to %s took: %s secs." % ( req.message_store_name, (tm1-tm0))

    return EpisodeServiceResponse(msg.header, msg.uuid, msg.soma_roi_id, msg.soma_map, \
                                  msg.soma_config, msg.start_time, msg.episodes)


def generate_episodes():

    rospy.init_node('episode_server')
                       #service_name      #service_prototype  #handler_function
    s = rospy.Service('/episode_service', EpisodeService,  handle_episodes)
    print "Ready to service some episodes..."
    rospy.spin()


if __name__ == "__main__":
    stitching = stitch_uuids()
    generate_episodes()
