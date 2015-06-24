#!/usr/bin/env python

#__author__      = "Paul Duckworth"
#__copyright__   = "Copyright 2015, University of Leeds"

import sys
import rospy
from std_msgs.msg import String
from relational_learner.srv import *
from relational_learner.msg import episodes_to_novelty
from human_trajectory.msg import Trajectories

import relational_learner.obtain_trajectories as ot


class EpisodeClient(object):

    def __init__(self, qsr_visualisation):
        #self.ret = None
        self.current_uuids_detected = []
        self.vis = qsr_visualisation
        self.pub = rospy.Publisher("/trajectory_behaviours/episodes", episodes_to_novelty, queue_size=10)
        rospy.Subscriber("/human_trajectories/trajectories/batch", Trajectories, self.callback)

        #rospy.Subscriber("/human_trajectories/trajectories/batch", , self.human_classifier)

    def episode_client(self, Trajectory):
        rospy.wait_for_service('/episode_service')
        proxy = rospy.ServiceProxy('/episode_service', EpisodeService)  
        req = EpisodeServiceRequest(Trajectory, self.vis, self.current_uuids_detected, \
                                    "relational_episodes")
        ret = proxy(req)
        return ret

    def callback(self, msg):
        if len(msg.trajectories) > 0:

            self.current_uuids_detected = []
            for trajectory in msg.trajectories:
                self.current_uuids_detected.append(trajectory.uuid)
            number_of_ids = len(self.current_uuids_detected)
            print "\n%s id(s) in scene = %s" % (number_of_ids, self.current_uuids_detected)

            for rotate_cnt, trajectory in enumerate(msg.trajectories):
                rotate_cnt+=1
 
                print "passing %s, %s." % (rotate_cnt, trajectory.uuid)
                ret = self.episode_client(trajectory)
                print "returned %s." % rotate_cnt

                if ret.soma_roi_id == "":
                    print "uuid = %s is outside of all soma roi" % trajectory.uuid
                    continue

                rotate_flg=True if rotate_cnt == number_of_ids else False
                print "rotate flag = ", rotate_flg

                p_msg = episodes_to_novelty(ret.header, ret.uuid, ret.soma_roi_id,  ret.soma_map, \
                    ret.soma_config, ret.start_time, ret.episodes, rotate_flg)

                self.pub.publish(p_msg)
                print "uuid = %s Episodes uploaded to database" % trajectory.uuid


if __name__ == "__main__":
    rospy.init_node('episodes_client')
    if len(sys.argv) == 2:
        vis=bool(sys.argv[1])
    else:
        vis=False
    print "Usage: QSR visualisation in RVIZ not selected. Turn_on = 1. 0 by default."
    print "qsr_viz = ", vis

    ec = EpisodeClient(vis)

    rospy.spin()
























