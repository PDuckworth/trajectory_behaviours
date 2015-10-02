#!/usr/bin/env python

"""Queries Mongodb for Object and Trajectory data"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"


import rospy
import pymongo
import os, sys, datetime, time, copy
import logging
import itertools
import numpy as np
import cPickle as pickle
import random, math
import landmark_utils  as lu
from scipy import spatial
import matplotlib.pyplot as plt
import relational_learner.learningArea as lA
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

from std_msgs.msg import String
from geometry_msgs.msg import Point,Pose
from relational_learner.srv import *
from relational_learner.msg import *
from human_trajectory.msg import Trajectory,Trajectories
from soma_trajectory.srv import TrajectoryQuery, TrajectoryQueryRequest, TrajectoryQueryResponse
from soma_geospatial_store.geospatial_store import GeoSpatialStoreProxy

#**************************************************************#
#             Obtain Objects and Trajectories                  #
#**************************************************************#
class query_objects():

    def __init__(self, query=None):
        self.all_objects = dict()
        host = rospy.get_param("mongodb_host")
        port = rospy.get_param("mongodb_port")
        self._client = pymongo.MongoClient(host, port)
        self._retrieve_logs(query)

    def _retrieve_logs(self, query=None):
        print query
        logs = self._client.message_store.soma.find(query)

        for log in logs:
            for i, _id in enumerate(log['id']):

                x = log['pose']['position']['x']
                y = log['pose']['position']['y']
                z = log['pose']['position']['z']
                obj_instance = log['type'] + '_' + log['id']

                if _id not in self.all_objects:
                    self.all_objects[obj_instance] = (x,y,z)
                else:
                    self.all_objects[obj_instance] = (x,y,z)
        return


    def check(self):
        print 'All objects in SOMA:'
        for i, key in enumerate(self.all_objects):
            print repr(i) + ",  " + repr(key) + ",  " + repr(self.all_objects[key])
        print repr(len(self.all_objects)) + ' objects Loaded.\n'


def filtered_trajectorys(just_ids=True, msg_store='people_trajectory', timedelta=None):

    """Find the completed (and filtered) trajectories from people_trajectory store"""
    store = GeoSpatialStoreProxy('message_store', msg_store)
    print "querying: ", msg_store

    if timedelta != None: query = {"_meta.inserted_at" : {"$lt": timedelta}}
    else: query = {"uuid": {"$exists": "True"}}

    rospy.loginfo("Query: %s" % query )
    res = store.find_projection(query, {"uuid":1, "trajectory":1})
    rospy.loginfo("Result: %s filtered trajectories" % res.count())

    if just_ids:
        rospy.loginfo("Getting UUIDs...")
        uuids=[]
        for full_trajectory in res:
            uuids.append(full_trajectory["uuid"])
        return uuids

    ##If Require Full Pose info:
    rospy.loginfo("Getting UUIDs and pose data...")
    ratios = {}
    uuid_pose_dict = {}
    for full_trajectory in res:

        uuid = full_trajectory["uuid"]
        x, y, quarts = [], [], []
        for cnt, entry in enumerate(full_trajectory["trajectory"]):
            x.append(entry["pose"]["position"]["x"])
            y.append(entry["pose"]["position"]["y"])
            #Quaternion = (entry["pose"]["orientation"]["y"], entry["pose"]["orientation"]["x"], \
            #    entry["pose"]["orientation"]["z"], entry["pose"]["orientation"]["w"])
            #quarts.append(Quaternion)
        number_of_poses = cnt+1
        distance = math.hypot((x[-1]-x[0]), (y[-1]-y[0]))
        ratio = distance/float(number_of_poses)
        uuid_pose_dict[uuid] = {"x":x, "y":y, \
                                #"quart":quarts,
                                "distance" : distance, "num_poses" : number_of_poses}

        if len(x) != len(y): print "X and Y dimensions are different!"
        ratios[uuid] = ratio
    return uuid_pose_dict, ratios


def trajectory_object_dist(objects, trajectory_poses, select=4):
    uuids=trajectory_poses.keys()
    object_ids=objects.keys()
    print "%s trajectories.  %s objects. Selecting closest %s..." % (len(uuids), len(object_ids), select)

    object_distances={}
    distance_objects={}
    for (uuid, obj) in itertools.product(uuids, object_ids):
        #object_distances[(uuid, obj)] = [] #No need for list, if only taking init_pose
        #print (uuid, obj)

        traj_init_pose = trajectory_poses[uuid][0][:2]  #Select the first trajectory pose for now

        if objects[obj] == None: continue               #If object is in Soma but not Geo/Soma it will not have coords
        object_pose = objects[obj][0:2]                 #Objects only have one pose
        dist = spatial.distance.pdist([traj_init_pose, object_pose], 'euclidean')

        if uuid not in object_distances:
            object_distances[uuid] = {}
            distance_objects[uuid]={}

        object_distances[uuid][obj] = dist[0]
        distance_objects[uuid][dist[0]]= obj
        if len(object_distances[uuid]) != len(distance_objects[uuid]):
            print "multiple objects exactly the same distance from trajectory: " + repr(uuid)
            print "object: " + repr(obj)
            continue

    closest_objects = {}
    for uuid, dist_objs in distance_objects.items():
        keys = dist_objs.keys()
        keys.sort()

        #select closest 4 (%select) objects or landmarks
        closest_dists = keys [0:select]
        closest_objects[uuid]=[]

        for dist in closest_dists:
            closest_objects[uuid].append(dist_objs[dist])

    return closest_objects


class QueryClient():
    def __init__(self):
        service_name = 'trajectory_query'
        rospy.wait_for_service(service_name)
        self.ser = rospy.ServiceProxy(service_name, TrajectoryQuery)

    def query(self, query, vis = False, vis_option = "random"):
        if vis: print "visualise colour option:", vis_option
        try:
            req = TrajectoryQueryRequest()
            req.query = query
            req.visualize = vis
            req.vis_option = vis_option
            res = self.ser(req)
            return res
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)


class query_trajectories():
    def __init__(self, query, vis=False, colour="direction_red_blue"):
        client = QueryClient()
        vis_option = colour
        self.res = client.query(query, vis, vis_option)
        self.uuids_list = []
        self.sequence_id_dict = {}
        for trajectory in self.res.trajectories.trajectories:
            self.uuids_list.append(trajectory.uuid)

        if self.res.error:
            rospy.logerr("Result: error: %s (Invalid query: %s)" % (self.res.error, query))
        else:
            if vis:
                print "Query returned: %s trajectories. " % repr(len(self.res.trajectories.trajectories))


    def get_best_trajectories(self, requested=0.1):
        print "Getting UUID and Pose info"
        ratios = {}
        ##Dont need all the pose data kept atm
        #uuid_pose_dict = {}
        poses = {}
        for full_trajectory in self.res.trajectories.trajectories:

            uuid = full_trajectory.uuid
            x, y = [], []
            for entry in full_trajectory.trajectory:
                x.append(entry.pose.position.x)
                y.append(entry.pose.position.y)

            number_of_poses = len(full_trajectory.trajectory)
            distance = math.hypot((x[-1]-x[0]), (y[-1]-y[0]))
            ratio = distance/float(number_of_poses)
            #uuid_pose_dict[uuid] = {"x":x, "y":y, \
            #                        "distance" : distance, "num_poses" : number_of_poses}
            #poses[uuid] = number_of_poses
            if number_of_poses < 30: ratio = 0
            ratios[uuid] = ratio

        print "Selecting best %s percent..." % (requested*100)

        a = list(reversed(sorted(ratios, key=ratios.get)))
        request = int(len(a) * requested)
        best_uuids = a[:request]
        print ratios[best_uuids[0]], ratios[best_uuids[10]], ratios[best_uuids[-1]]
        print len(best_uuids)
        self.uuids_list=[]
        self.filtered_trajs = []

        for trajectory in self.res.trajectories.trajectories:
            if trajectory.uuid in best_uuids:
                self.filtered_trajs.append(trajectory)
                self.uuids_list.append(trajectory.uuid )
        return poses

    def get_poses(self):
        self.trajs = {}
        self.trajectory_times = []

        for trajectory in self.res.trajectories.trajectories:
            self.sequence_id_dict[trajectory.uuid] = trajectory.sequence_id
            self.trajs[trajectory.uuid] = []
            self.trajectory_times.append(trajectory.start_time.secs) # Temporal Info
            for entry in trajectory.trajectory:
                x=entry.pose.position.x
                y=entry.pose.position.y
                z=entry.pose.position.z
                self.trajs[trajectory.uuid].append((x,y,z))

def convert_keys_to_string(dictionary):
    """Recursively converts dictionary keys to strings."""
    if isinstance(dictionary, unicode):
        return str(dictionary)
    elif isinstance(dictionary, list):
        return dictionary
    return dict((str(k), convert_keys_to_string(v))
        for k, v in dictionary.items())


class EpisodeClient(object):

    def __init__(self, msg_store = "relational_episodes"):
        self.ret = None
        self.current_uuids_detected = []
        self.msg_store = msg_store

    def episode_client(self, Trajectory):
        rospy.wait_for_service('/episode_service')
        proxy = rospy.ServiceProxy('/episode_service', EpisodeService)
        req = EpisodeServiceRequest(Trajectory, False, self.current_uuids_detected, \
                                    self.msg_store)
        ret = proxy(req)
        return ret



def make_query(uuids):
    """make a people_trajectory 'trajectory_query_service' query from a list of uuids"""

    if len(uuids)<1: return  None
    aa = ['"%s"' %i for i in uuids]
    query = '''{"uuid" :{ "$in": ['''
    for i in aa[:(len(aa)-1)]:
        query = query+i+", "
    query = query+aa[-1]+"] }}"
    #print "query = ", query
    #query = '''{"uuid" :{ "$in": [ "0c85ef13-51c1-5ce7-b574-b788ed399308", "752e4b4e-d0f5-5f15-b11a-8afa13c6a883"] }}'''
    return query


def get_uuids_from_traj_store(request_percent = 0.1, traj_store='people_trajectory', \
            data_dir= '/home/strands/STRANDS/', vis=False, end_date=None):
    s = time.time()

    dict_of_poses, ratios = filtered_trajectorys(just_ids=False, msg_store=traj_store, timedelta=end_date)

    both = (dict_of_poses, ratios)

    #both = pickle.load(open(data_dir+"trajectory_dump/tuple_of_all_traj_info.p", "r"))
    pickle.dump(both, open(data_dir+"trajectory_dump/tuple_of_all_traj_info.p", "w"))
    print "number of trajectories dumped = %s" % len(ratios)

    a = list(reversed(sorted(ratios, key=ratios.get)))
    request = int(len(a) * request_percent)
    best_uuids = a[:request]

    print ratios[best_uuids[0]], ratios[best_uuids[10]], ratios[best_uuids[50]]
    print "best %s selected (sorted by highest ratio)" % request
    print "time = ", time.time() - s

    return best_uuids


if __name__ == "__main__":
    global __out
    __out = True
    rospy.init_node("trajectory_obtainer")

    data_dir= '/home/strands/STRANDS/'
    soma_map = 'g4s'
    soma_config = 'g4s_landmarks'

    ## Filter All Trajectories in message store
    #request = 1.0
    traj_store = 'people_trajectory'
    episode_store = 'relational_episodes_11_12'

    #best_uuids_all_roi = get_uuids_from_traj_store(request_percent=request,  traj_store= traj_store, vis=True)

    rospy.loginfo("Running trajectoy/ROI query ")
    gs = GeoSpatialStoreProxy('geospatial_store','soma')
    cnt=0

    #Query two days data at a time.
    t1 = datetime.datetime(2015, 5, 20, 00, 0, 0, 000000)
    t2 = datetime.datetime(2015, 5, 23, 00, 0, 0, 000000)
    #t2 = datetime.datetime(2015, 5, 23, 00, 0, 0, 000000)
    query_start_seconds = (t1-datetime.datetime(1970,1,1)).total_seconds()
    query_end_seconds = (t2-datetime.datetime(1970,1,1)).total_seconds()
    print "query time: %s - %s " % (t1, t2)

    for roi in gs.roi_ids(soma_map, soma_config):
        if roi != '1': continue
        cnt+=1
        print 'ROI: ', gs.type_of_roi(roi, soma_map, soma_config), roi
        geom = gs.geom_of_roi(roi, soma_map, soma_config)
        res = gs.objs_within_roi(geom, soma_map, soma_config)
        if res == None:
            print "No Objects in this Region"
            continue
        objects_in_roi = {}
        print "Objects: "
        for i in res:
            key = i['type'] +'_'+ i['soma_id']
            objects_in_roi[key] = i['loc']['coordinates']
            print key, objects_in_roi[key]

        #geom_str = convert_keys_to_string(geom)
        #query = '''{"loc": { "$geoWithin": { "$geometry":
        query = '''{"loc": { "$geoIntersects": { "$geometry":
        { "type" : "Polygon", "coordinates" : %s }}}, "start": {"$lt": %s, "$gte":%s}}''' % (geom['coordinates'], query_end_seconds, query_start_seconds )

        st = time.time()
        print "location and time query: \n ", query

        #Querying for data in this region
        q = query_trajectories(query, False, "direction_red_green")
        l = len(q.uuids_list)
        percent = 0.05
        request = int(l*percent)
        print "number requested in roi = ", request
        q.get_best_trajectories(requested = percent)
        print "Takes: ", time.time() - st

        vis = True
        if vis:
            print len(q.uuids_list)
            query = make_query(q.uuids_list)
            q2 = query_trajectories(query, True, "direction_red_green")

        if len(q.uuids_list)==0:
            print "No Trajectories in this Region"
            continue
        else:
            ec = EpisodeClient(msg_store = episode_store)
            #ec.current_uuids_detected = q.uuids_list
            print "Calling the Episode Service to upload episodes to %s" % episode_store
            ec.current_uuids_detected = []
            for cnt, trajectory in enumerate(q.filtered_trajs):
                print cnt, trajectory.uuid

                ret = ec.episode_client(trajectory)


            #traj_file = os.path.join(data_dir + 'trajectory_dump/using_obj_ids_roi_%s_end2.p') % roi
            #pickle.dump(q.uuids_list, open(traj_file, 'w'))
            #print "saved trajectory dict. here: %s" % traj_file
            #print "number of unique traj returned = " + repr(len(q.uuids_list))
            #print "%s percent filtered on displacement = %s trajectories" % (request*100, len(q.uuids_list))

        ##Create Landmark pins at randomly selected poses from all the trajectory data
        #all_poses = list(itertools.chain.from_iterable(q.trajs.values()))
        #print "number of poses in total = " +repr(len(all_poses))

        ##To Dump trajectories for testing
        #data_dir='/home/strands/STRANDS'
        #obj_file  = os.path.join(data_dir, 'obj_dump.p')

        #pickle.dump(objects_in_roi, open(obj_file, 'w'))
        #pickle.dump(q.trajs, open(traj_file, 'w'))

        #pins = lu.Landmarks(select_landmark_poses(all_poses))
        #static_things = pins.poses_landmarks
        #static_things = objects_in_roi
        #objects_per_trajectory = trajectory_object_dist(static_things, trajectory_poses)
