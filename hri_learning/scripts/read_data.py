#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os, sys
import copy
import csv
import numpy as np
import cPickle as pickle
from qsrlib_io.world_trace import Object_State, World_Trace
from qsrlib_ros.qsrlib_ros_client import QSRlib_ROS_Client
from qsrlib.qsrlib import QSRlib, QSRlib_Request_Message
import qsrlib_qstag.utils as utils
import qsrlib_utils.median_filter as filters

class DataReader(object):

    template = {
        "agent1": {
            "name": "",
            "x": np.array([]),
            "y": np.array([])
        },
        "agent2": {
            "name": "",
            "x": np.array([]),
            "y": np.array([])
        },
        "agent3": {
            "name": "",
            "x": np.array([]),
            "y": np.array([])
        }
    }

    def generate_data_from_input(self, path, exclude=("stats.csv",)):
        ret = []

        for f in os.listdir(path):
            if f.endswith(".csv") and not f in exclude:
                data = copy.deepcopy(self.template)
                filename = path + '/' + f
                with open(filename) as csvfile:
                    reader = csv.DictReader(csvfile)
                    rospy.loginfo("Reading file: '%s'" % f)
                    for idx,row in enumerate(reader):
                        if data["agent1"]["name"] == "":
                            data["agent1"]["name"] = row["agent1"]
                        if data["agent2"]["name"] == "":
                            data["agent2"]["name"] = row["agent2"]
                        if data["agent3"]["name"] == "":
                            data["agent3"]["name"] = row["agent3"]

                        data["agent1"]["x"] = np.append(data["agent1"]["x"], float(row["x1"]))
                        data["agent1"]["y"] = np.append(data["agent1"]["y"], float(row["y1"]))

                        data["agent2"]["x"] = np.append(data["agent2"]["x"], float(row["x2"]))
                        data["agent2"]["y"] = np.append(data["agent2"]["y"], float(row["y2"]))

                        data["agent3"]["x"] = np.append(data["agent3"]["x"], float(row["x3"]))
                        data["agent3"]["y"] = np.append(data["agent3"]["y"], float(row["y3"]))

                    ret.append(data)

        return ret

    def convert_to_world(self, data_dict, use_every=1):
        world = World_Trace()

        ob = []
        for k, v in data_dict.items():
            count = 0
            for idx, (x, y) in enumerate(zip(np.array(v["x"], dtype=float), np.array(v["y"], dtype=float))):
                #if idx % 5 != 0: continue

                ob.append(Object_State(
                    name=v["name"],
                    timestamp=count,
                    x=x,
                    y=y
                ))
                count+=1
            world.add_object_state_series(ob)
        return world

def pretty_print_world_qsr_trace(which_qsr, qsrlib_response_message):
	print(which_qsr, "request was made at ", str(qsrlib_response_message.req_made_at)
		  + " and received at " + str(qsrlib_response_message.req_received_at)
		  + " and finished at " + str(qsrlib_response_message.req_finished_at))
	print("---")
	print(qsrlib_response_message.qsrs.get_sorted_timestamps())
	print("Response is:")
	for t in qsrlib_response_message.qsrs.get_sorted_timestamps():
		foo = str(t) + ": "
		for k, v in zip(qsrlib_response_message.qsrs.trace[t].qsrs.keys(),
						qsrlib_response_message.qsrs.trace[t].qsrs.values()):
			foo += str(k) + ":" + str(v.qsr) + "; "
		print(foo)


if __name__=="__main__":
    rospy.init_node("data_reader")
    dr = DataReader()
    data_list = dr.generate_data_from_input(sys.argv[1])

    #which_qsr = ["qtcbs", "rcc3", "argd"]
    which_qsr = ["qtcbs"]

    qtcbs_qsrs_for = [("robot", "human")]#[("human", "robot"),("robot", "goal"),("human", "goal")]
    rcc3_qsrs_for = [("robot", "human")]
    argd_qsrs_for = [("robot", "goal")]

    quantisation_factor = 0.01
    validate = False
    no_collapse = True

    object_types = {"human": "human",
					"robot": "robot",
                    "goal": "goal"}

    params = {"min_rows" : 1, "max_rows" : 1, "max_eps" : 5}

    dynamic_args = {"qtcbs": {"quantisation_factor": quantisation_factor,
							  "validate": validate,
							  "no_collapse": no_collapse,
                              "qsrs_for": qtcbs_qsrs_for
							 },
                    "rcc3": {"qsrs_for": rcc3_qsrs_for},
                    "argd": {"qsrs_for": argd_qsrs_for,
                             "qsr_relations_and_values" : {"Touch": 0.5, "Near": 2, "Far": 3}},
                    "qstag": {"object_types" : object_types,
                              "params" : params},

                    "filters": {"median_filter" : {"window": 2}}
					}

    for cnt, data_dict in enumerate(data_list):
        world = dr.convert_to_world(data_dict, 1)
        if cnt > 0 : continue
        print "subject:", cnt, "frames: ", len(world.get_sorted_timestamps())

        print "requesting everything..."
        qsrlib_request_message = QSRlib_Request_Message(which_qsr=which_qsr, input_data=world, \
                                                         dynamic_args=dynamic_args)
        # cln = QSRlib_ROS_Client()
        # req = cln.make_ros_request_message(qsrlib_request_message)
        # res = cln.request_qsrs(req)
        # qsrlib_response_message = pickle.loads(res.data)
        qsrlib = QSRlib()
        qsrlib_response_message = qsrlib.request_qsrs(req_msg=qsrlib_request_message)

        print "checking qstag..."

        qstag = qsrlib_response_message.qstag

        print "Episodes:"
        for i in qstag.episodes:
            print i

        print "Graphlets:"
        for i, j in qstag.graphlets.graphlets.items():
            print "\n", j

        print dir(qstag.graphlets)



    	utils.graph2dot(qstag, "/tmp/graph.dot")
    	os.system('dot -Tpng /tmp/graph.dot -o /tmp/graph.png')
