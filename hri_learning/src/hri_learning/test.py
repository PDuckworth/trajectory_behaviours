#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os, sys
import copy
import csv
import numpy as np
from qsrlib_io.world_trace import Object_State, World_Trace


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

    def convert_to_world(self, data_dict):
        world = World_Trace()

        ob = []
        for k, v in data_dict.items():
            for idx, (x, y) in enumerate(zip(np.array(v["x"], dtype=float), np.array(v["y"], dtype=float))):
                ob.append(Object_State(
                    name=v["name"],
                    timestamp=idx,
                    x=x,
                    y=y
                ))

            world.add_object_state_series(ob)

        return world

if __name__=="__main__":
    rospy.init_node("data_reader")
    dr = DataReader()
    print "hello world"
    sys.exit(1)
    data_list = dr.generate_data_from_input(sys.argv[1])

    for data_dict in data_list:
        world = dr.convert_to_world(data_dict)
        print world

