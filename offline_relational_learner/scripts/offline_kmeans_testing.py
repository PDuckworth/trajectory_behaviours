#!/usr/bin/env python

"""visualise_qualitative_predictions.py:"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"
import os, sys, time, copy
import argparse
import rospy
import numpy as np
import cPickle as pickle
from nav_msgs.msg import GridCells, OccupancyGrid

import novelTrajectories.config_utils as util
import offline_relational_learner.utils as o_utils
import offline_relational_learner.visualise_qualitative_predictions as vis_tools

from relational_learner.learningArea import Learning
import relational_learner.obtain_trajectories as ot

import matplotlib.pyplot as plt
from sklearn.metrics import (precision_score, recall_score, f1_score, classification_report)

def kmeans_test_set(smartThing, iter, vis=False, publish=None):
    """Test a bunch of histograms against the learnt Kmeans model"""

    estimator = smartThing.methods["kmeans"]
    test_set_distances = []
    novel_uuids, not_novel = [], []

    #Iterate over the split-trajectories. To predict future qualitative movements
    if iter:
        k_means_iterate_over_examples(smartThing, publish, vis)
    else:
        for uuid, test_histogram in smartThing.X_test.items():
            closest_cluster_id = estimator.predict(test_histogram)
            closest_cluster = estimator.cluster_centers_[closest_cluster_id]
            dst = distance.euclidean(test_histogram, closest_cluster)
            test_set_distances.append(dst)

            mean = estimator.cluster_dist_means[closest_cluster_id[0]]
            std = estimator.cluster_dist_std[closest_cluster_id[0]]

            if dst > mean + (std): novel_uuids.append(uuid)
            else: not_novel.append(uuid)

        print "Inertia of Test Set = %s\n" % sum(test_set_distances)
        return test_set_distances, (novel_uuids, not_novel)


def k_means_iterate_over_examples(smartThing, publishers, vis=False):

    estimator = smartThing.methods["kmeans"]
    graphlet_book = smartThing.graphlet_book

    rospy.loginfo('Testing sequences and comparing with complete trajectories...')

    #1. Create an empty GridCells
    grid = vis_tools.Object_gridCells()
    #2. Add all objects from SOMA into the Cells Grid
    grid.add_all_objects_to_grid()

    #3. Get the QSR and Object params - Create an array Mask for each QSR
    qsrs = vis_tools.qsr_param_masks(grid, dbg=False)

    #4. Get the cluster centers represented as Activity Graphs
    #   Produce a mask for each cluster center from the Graphs
    qsrs.get_graph_clusters_from_kmeans(estimator, graphlet_book)
    print "Created an Occupency Grid for clusters: ", qsrs.cluster_occu.keys()

    if vis:
        visualise_clusters(qsrs, publishers, smartThing.show_in_rviz) #Currently all traj's belonging to cluster in this list

    #6. Test:
    rospy.loginfo('Testing all the sequences of trajectories (takes a while...)')

    collect_seq_histograms={}
    for uuid_seq_string, histogram in smartThing.X_test.items():
        if "_test_seq_" not in uuid_seq_string: continue
        uuid, seq = uuid_seq_string.split('_')[0], uuid_seq_string.split('_')[3]

        if uuid not in collect_seq_histograms: collect_seq_histograms[uuid] = {}
        collect_seq_histograms[uuid][int(seq)] = histogram


    """HOW HAS THIS HAPPENED?"""
    #for i, v in collect_seq_histograms["eb78e131-d6ce-5ce4-ad21-7649f71981d9"].items():
    #    print i

    """Two scores:
    #1. How good is the prediction on partial trajectories. Does seq match seq_n's prediction.
    #2. How good is the prediction of future trajectories. This is the occupancygrid scores summed for future.
    """
    cluster_scores = {}
    for uuid, sequence_preds in collect_seq_histograms.items():
        print uuid
        cluster_scores[uuid] = {}
        #if uuid != "eb78e131-d6ce-5ce4-ad21-7649f71981d9": continue

        #1. Get trajectory from mongo
        query = '''{"uuid" : "%s" }''' % uuid
        q = ot.query_trajectories(query, vis)
        q.get_poses()

        max_seq = max(sequence_preds.keys())
        for seq in xrange(1, len(sequence_preds)+1):
            if seq not in sequence_preds: continue
            cluster_scores[uuid][seq] = []

            hist = np.array(sequence_preds[seq])
            cluster_id = estimator.predict(hist)[0]

            qsrs_copy = copy.deepcopy(qsrs.cluster_occu[str(cluster_id)])
            score = get_score_from_occupencygrid(q.trajs[uuid], (seq, max_seq), \
                                        qsrs_copy, cluster_id, vis, publishers[1])
            cluster_scores[uuid][seq].append((cluster_id, score))
        #print cluster_scores

    pickle.dump(cluster_scores, open('/home/strands/STRANDS/TESTING/cluster_scores_using_weights.p', "w"))
    rospy.loginfo("Finished generating scores (and dumped them in TESTING)")

    """
    MULTIPLE "SCORES"?
        1. NUMBER OF PREDICTED GRAPHLETS ALSO IN CLUSTER CENTERS
        2. NUMBER OF GRAPHLETS IN CLUSTER WHICH ARE IN PREDICTION
        3. AMMOUNT THE (predicted) OCCUPANCY GRID OVERLAPS THE FULL TRAJECTORY -> possibly the best metric.
    """


def get_score_from_occupencygrid(traj, seq_range, qsrs, cluster_id, vis=False, pub=None):
    (seq, max_seq) = seq_range
    mini_batches = len(traj)/ float(max_seq)

    traj_occu_values = []
    next_pose = seq*int(mini_batches) + 1
    print "seq: %s (of %s)" %(seq, max_seq)
    print "  predicted cluster: %s. #test_poses: %s. #future_poses: %s." \
            %(cluster_id, next_pose, len(traj[next_pose:]))
    if len(traj[next_pose:]) == 0: return

    visualise_points=[]
    for cnt, (x,y,z) in enumerate(traj):
        occu_pt = vis_tools.xy_to_occu(x,y)
        traj_occu_values.append(qsrs.occupancygrid.data[occu_pt])
        if vis: visualise_points.append(occu_pt)

    if vis:
        for cnt, occu_pt in enumerate(visualise_points):
            if cnt < next_pose: qsrs.occupancygrid.data[occu_pt] = 120
            else: qsrs.occupancygrid.data[occu_pt] = 0

    #print "trajectory prediction scores = ", traj_occu_values
    avg = sum(traj_occu_values[next_pose:])/float(len(traj_occu_values[next_pose:]))
    print "  avg future occu score = %s" %avg

    if vis: visualise_predictions(qsrs, cluster_id, pub)
    return avg


def visualise_predictions(qsrs, id, pub):
    """TECHNIQUES TO IMPOVE VIS:
        1. USE THRESHOLD to remove yellow (low)
        2. USE VIEW-CONE to localise the prediction."""
    cnt=0
    while not rospy.is_shutdown() and cnt==0:
        qsrs.pub_occu(pub)
        print "showing: cluster_%s" % id
        raw_input("pause...")
        cnt = 1

def visualise_clusters(qsrs, pub, show_uuids):
    cnt=0
    while not rospy.is_shutdown() and cnt==0:
        for cluster in qsrs.cluster_occu:
            print "Cluster: %s" %cluster
            query = ot.make_query(show_uuids[cluster])
            q = ot.query_trajectories(query, True)

            qsrs.cluster_occu[cluster].pub_occu(pub[1])
            qsrs.cluster_occu[cluster].grid.pub_grid(pub[0])
            raw_input("pause...")
        cnt = 1
        print "finished showing off clusters"


def getFlooredThreshold(a, MinClip):
    return np.floor(float(a) / MinClip) * MinClip

def getCeilThreshold(a, MinClip):
    if a == 100: return 110
    else: return int(np.ceil(float(a) / MinClip) * MinClip)

def evaluate_predictions(scores_dict, vis=False, dbg=False):
    actual = {}
    predicted = {}
    scores = {}

    for uuid, seq_dict in scores_dict.items():
        if len(seq_dict.keys()) >0: final_seq = max(seq_dict.keys())
        else: continue

        (final_prediction, final_score) = seq_dict[final_seq][0]
        if dbg: print "%s. Final Pred: %s" % (uuid, final_prediction)

        for seq, tuple in seq_dict.items():
            (prediction, score) = tuple[0]
            percent_seen = seq/float(final_seq)*100
            ceil_value = getCeilThreshold(percent_seen, 10)
            if dbg: print "  seq: %s, pred: %s, score: %s, floored: %s" % (seq, prediction, score, ceil_value)

            if int(ceil_value) not in actual:
                actual[ceil_value]=[final_prediction]
                predicted[ceil_value]=[prediction]
                scores[ceil_value]=[score]
            else:
                actual[ceil_value].append(final_prediction)
                predicted[ceil_value].append(prediction)
                scores[ceil_value].append(score)

    results = (predicted, actual, scores)
    if vis: plot_results(results)


def generate_fig(x, y, label, yticks):
    percent_map = {10:'<10%',20:'<20%',30:'<30%',40:'<40%',50:'<50%',
                60:'<60%',70:'<70%',80:'<80%',90:'<90%',100:'<100%',110:'All'}

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(x,y)
    ax.set_ylabel(label)
    ax.set_xlabel("percentage of trajectory seen")
    ax.set_xticks(range(10,120, 10))
    ax.set_xticklabels([percent_map[i] for i in range(10,120, 10) ])
    if yticks is 0.1:
        ax.set_yticks([0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1])
    elif yticks is 10:
        ax.set_yticks(range(60,85,5))

    plt.title('%s: \nPredicting the trajectory cluster on partial trajectories' %label)
    filename = '/tmp/%s_cluster_scores.jpg' % label
    plt.savefig(filename, bbox_inches='tight', dpi=100)


def plot_results(results):
    import warnings
    warnings.filterwarnings("ignore", category=DeprecationWarning)
    rospy.loginfo('Getting Results...')

    (y_pred, y_true, scores) = results
    p, r, f1, avg_scores = [], [], [], []

    for percent in xrange(10, 120, 10):
        p.append(precision_score(y_true[percent], y_pred[percent]))
        r.append(recall_score(y_true[percent], y_pred[percent]))
        f1.append(f1_score(y_true[percent], y_pred[percent]))

        if percent != 110:
            avg = sum(scores[percent])/float(len(scores[percent]))
            avg_scores.append(avg*-1)

        print "Seq percent: %s" % percent
        print "Prediction score %s" % avg
        print(classification_report(y_true[percent], y_pred[percent]))

    x = range(10, 120, 10)
    generate_fig(x, p, "Precision", 0.1)
    generate_fig(x, r, "Recall", 0.1)
    generate_fig(x, f1, "F1 Score", 0.1)

    x = range(10, 110, 10)
    generate_fig(x, avg_scores, "Future Prediction Score (avg)", 10)


    plt.show()



if __name__ == "__main__":
    rospy.init_node('TestingKmeans')
    data_dir = '/home/strands/STRANDS/'

    parser = argparse.ArgumentParser(description="Unsupervised Learning on relational qsr epiosdes")
    parser.add_argument("-t", "--test", type=str, help=" 'TestSeq', 'TestFull', or 'None'", default=None)
    parser.add_argument("-p", "--plotting", type=int, help="plot grap", default=0)
    parser.add_argument("-v", "--vis_pred", type=int, help="visualise predictions in rviz", default=0)
    args = parser.parse_args()

    filename = os.path.join(data_dir + 'learning/roi_1_smartThing_4weeksTrain.p')
    smartThing=Learning(load_from_file=filename)

    pub_g = rospy.Publisher('/trajectory_behaviours/grid', GridCells, latch=True, queue_size=0)
    pub_o = rospy.Publisher('/trajectory_behaviours/occu', OccupancyGrid, latch=True, queue_size=0)
    publishers = (pub_g, pub_o)

    if args.test == "TestSeq":
        kmeans_test_set(smartThing, iter=True, vis= bool(args.vis_pred), publish=publishers)
    elif args.test == "TestFull":
        test_set_distances, (novel_uuids, not_novel) = kmeans_test_set(smartThing, iter=True, vis= bool(args.vis_pred), publish=publishers)


    test_scores = '/home/strands/STRANDS/TESTING/cluster_scores_using_weights.p'
    with open(test_scores, "rb") as f:
        scores = pickle.load(f)

    evaluate_predictions(scores, bool(args.plotting), dbg=True)
