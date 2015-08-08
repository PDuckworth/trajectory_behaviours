#!/usr/bin/env python

"""split_trajectory_test_set.py: splits trajectories in people_trajecory into mini-batch format
    and calls the Episode Service to create QSRs and store them on mongodb."""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"
import os, sys
import rospy
import copy
import cPickle as pickle
import time
from human_trajectory.trajectory import Trajectory
from human_trajectory.trajectories import *
from human_trajectory.msg import Trajectories
import relational_learner.obtain_trajectories as ot
from offline_episodes import OfflineEpisodes

def get_mini_batch_trajs(X_test, vis=False):
    rospy.loginfo('Calculating the mini-batch trajectories for the test set')
    st = time.time()

    """
    ## This method returns the split trajectories not as trajectory message types
    query = ot.make_query(X_test)
    q = ot.query_trajectories(str(query), bool(vis), "direction_red_green")
    q.get_poses()

    batch_trajs = {}
    for uuid, traj in q.trajs.items():
        l = int(len(traj)/ q.sequence_id_dict[uuid])

        batch_trajs[uuid] = []
        for i in xrange(1, q.sequence_id_dict[uuid]+1):
            batch_trajs[uuid].append(traj[:int(i*l)])
        batch_trajs[uuid].append(traj)
    """

    rospy.loginfo('Ferdi\'s method')
    query = {"uuid" :{ "$in" : X_test} }
    test_set = OfflineTrajectories(query, size=3000)

    batched_test_set = {}
    for uuid, trajectory in test_set.traj.items():
        trajectoryMsg = trajectory.get_trajectory_message()
        batched_test_set[uuid] = []
        l = int(len(trajectory.humrobpose)/ trajectory.sequence_id)
        print "%s. Len = %s. Sequences = %s. Chunk size = %s" \
            %(uuid, len(trajectory.humrobpose), trajectory.sequence_id, l)

        for i in xrange(1, trajectory.sequence_id+1):
            new = copy.deepcopy(trajectory)
            new.humrobpose = new.humrobpose[:int(i*l)]
            new._calc_length()
            new_traj_msg = new.get_trajectory_message(chunked=True)
            new_traj_msg.sequence_id = i
            batched_test_set[uuid].append(new_traj_msg)

        batched_test_set[uuid].append(trajectoryMsg)

    print "Time taken to re-create mini-batches: %0.3f seconds" % float(time.time()-st)
    return batched_test_set


def get_episodes_for_mini_batches(mini_batches):

    OE = OfflineEpisodes(msg_store='episodes_f1_multi_seq_test')

    for uuid, list_of_trajs in mini_batches.items():
        for cnt, traj in enumerate(list_of_trajs):
            print "\n", cnt, traj.uuid, traj.sequence_id, len(list_of_trajs)

            if cnt+1 == len(list_of_trajs): traj.sequence_id = len(list_of_trajs)
            traj.uuid = traj.uuid + "_test_seq_"+repr(traj.sequence_id)
            print traj.uuid
            ret = OE._episode_client(uuid, traj)


if __name__ == "__main__":
    rospy.init_node('splitting_trajectories')
    plotting=True
    data_dir = '/home/strands/STRANDS/'
    file_ = os.path.join(data_dir + 'TESTING/roi_1_week5_uuids.p')
    print file_
    test_set = pickle.load(open(file_, "r"))

    dict_of_mini_batches = get_mini_batch_trajs(test_set, vis=plotting)
    #print "RETURNS:", dict_of_mini_batches[reduced_test.keys()[0]]

    test_set = get_episodes_for_mini_batches(dict_of_mini_batches)

    cnt=0
    for k,v in dict_of_mini_batches.items():
        cnt+=len(v)
    print "There should be %s records in database with _test_seq_ in the UUID" % cnt

    #(load_from_file=file_)
