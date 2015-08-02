#!/usr/bin/env python

"""visualise_qualitative_predictions.py:"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"
import os, sys
import argparse
import rospy
import copy
import numpy as np
import cPickle as pickle
import time

from std_msgs.msg import Header, String
from geometry_msgs.msg import Pose, Point, PointStamped, Quaternion
from nav_msgs.msg import GridCells, OccupancyGrid
import novelTrajectories.config_utils as util

import offline_relational_learner.utils as o_utils

class visualise_grid(object):

    def __init__(self):
        self.setupGrid()
        self.setupOccupancyGrid()

    def setupGrid(self):
        """Grid is in map space coordinates."""
        self.grid = GridCells()
        self.grid.header.stamp = rospy.Time.now()
        self.grid.header.frame_id = "frame"
        self.grid.cell_width = 0.25
        self.grid.cell_height = 0.25
        self.grid.cells = []

    def pub_grid(self, pub):
        try:
            pub.publish(self.grid)
        except rospy.ROSInterruptException:
            pass


    def add_objects_to_grid(self, objects):
        for cnt, (object, pose) in enumerate(objects.all_objects.items()):
            p = Point()
            p.x = pose[0]
            p.y = pose[1]
            p.z = 0.1 #pose[2]
            self.grid.cells.append(p)

            id= object.split("_")[-1]
            if id == "32":
                self.test_object = object
                self.test_point = p


    def setupOccupancyGrid(self, width=657, height= 731, res=0.05):
        self.occupancygrid = OccupancyGrid()
        self.occupancygrid.data = [0]*int(width*height)
        self.occupancygrid.header = Header()
        self.occupancygrid.header.stamp = rospy.Time.now()
        self.occupancygrid.header.frame_id = "frame"
        self.occupancygrid.info.resolution = res
        self.occupancygrid.info.width = width
        self.occupancygrid.info.height = height
        p = Point()
        q = Quaternion()
        p.x=-6.3
        p.y=-6.55
        p.z = q.x = q.y = q.z = 0
        q.w = 1
        self.occupancygrid.info.origin = Pose(p,q)

    def occu_to_matrix(self):
        mat = np.mat(self.occupancygrid.data)
        mat = mat.reshape(self.occupancygrid.info.height, self.occupancygrid.info.width)
        #print "mat shape = ", mat.shape
        return mat

    def matrix_to_occu(self, mat):
        mat = mat.reshape(1, self.occupancygrid.info.width * self.occupancygrid.info.height, order='C')
        self.occupancygrid.data = mat.ravel().tolist()[0]
        print "len of occu grid = ", len(self.occupancygrid.data)

    def pub_occu(self, pub):
        try:
            pub.publish(self.occupancygrid)
        except rospy.ROSInterruptException:
            pass

class qsr_param_masks(object):

    def __init__(self, qsr_params, dbg, colour_range=0, res=0.05):

        self.qsr = qsr_params[0]
        self.qsr_params = qsr_params[1]
        self.res = res
        self.dbg = dbg
        self.colour_range = colour_range
        self.binary = False

        self.masks = {}
        self.binary_masks = {}

        print "QSRs requested: %s" % self.qsr

        if self.qsr == "arg_distance":
            rospy.loginfo('GENERATING QSR MASKS...')
            self.get_sorted_params()
            self.get_qsr_masks()


    def get_qsr_masks(self):

        for i in xrange(0,len(self.sorted_params)):
            #if i>3: continue
            print "\nLOOP", i
            cells = self.sorted_params[i][1] / self.res
            label = self.sorted_params[i][0]

            ##Make Touch and Near small enough to see in terminal :)
            #if i == 0: cells = 3
            #elif i == 1: cells = 5
            print "cells = ", cells
            self.masks[label] = self.create_circle_mask(cells, i)

    def get_sorted_params(self):
        self.sorted_params = []
        new_dict = dict (zip(self.qsr_params.values(), self.qsr_params.keys()))
        keys = new_dict.keys()
        keys.sort(key=int)
        for i in keys:
            self.sorted_params.append((new_dict[i], i))
        print self.sorted_params
        self.number_of_qsrs = len(self.sorted_params)


    def remove_center_of_mask(self, i, binary_mask):
        label = self.sorted_params[i][0]
        print "circle mask shape = ",  binary_mask.shape

        #For each qsr smaller than i, remove the mask (which correspond to the center of the donut).
        remove_i = i
        first_in_loop=True
        while remove_i > 0:
            remove_i-=1
            print "removing %s: %s" % (remove_i, self.sorted_params[remove_i][0])
            one_smaller_mask = self.binary_masks[self.sorted_params[remove_i][0]]
            binary_mask = o_utils.addAtPos(binary_mask, one_smaller_mask, (0,0), add=False, vis=self.dbg, first=first_in_loop)
            first_in_loop=False

        return binary_mask

    def create_circle_mask(self, cells, i):
        """Creates circles for each QSR. Then removes each smaller QSR mask from the centre.
        Currently doesn't: donut = np.logical_and(circle < (6400 + 60), circle > (6400 - 60))"""
        xx, yy = np.mgrid[:(cells*2)+1, : (cells*2)+1]
        qsr_mask = (xx - float(cells))** 2 + (yy - float(cells))** 2
        #print qsr_mask
        qsr_mask[qsr_mask > cells**2] = (cells**2)+1
        #print "max: ", qsr_mask.max()
        #print qsr_mask

        label = self.sorted_params[i][0]
        binary_circle_mask = (qsr_mask <=cells**2)*100
        #donut = np.logical_and(circle < (6400 + 60), circle > (6400 - 60)) ???

        #This removes the center of the mask if i>0
        self.binary_masks[label] = self.remove_center_of_mask(i, binary_circle_mask)

        #normalised_mask = ((mat.max() - mat) /mat.max())*-100
        #print normalised_mask

        """FOR LATER:#qsr_mask[abs(qsr_mask) <= 2.0] = 0  # -ve 1 and -ve 2 are grey in an occupencygrid colour scheme"""


def xy_to_occu(x,y, res=0.05, map_width=657, origin_x=6.3, origin_y=6.55):
    """(x,y) to index in the occupencygrid data vector"""
    return int((x+origin_x)/float(res)) + int(y/float(res) + origin_y/float(res))*map_width

def xy_to_mat(x,y, res=0.05, map_width=657, origin_x=6.3, origin_y=6.55, dbg=False):
    """(x,y) are (j,i). Returns (i,j) therefore: (y,x)"""
    if dbg:
        print "origin in mat:", int((origin_y/0.05)),  int(np.ceil(origin_x/0.05))
        print "i,j of point from (0,0):", int((y/0.05)), int(np.ceil(x/0.05))
        print "summed together:", int((y/0.05)) + int((origin_y/0.05)), int(np.ceil(x/0.05)+int(np.ceil(origin_x/0.05)))
    return int((y/0.05)) + int((origin_y/0.05)), int(np.ceil(x/0.05)+int(np.ceil(origin_x/0.05)))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Unsupervised Learning on relational qsr epiosdes")
    parser.add_argument("-p", "--plotting", help="plotting", default=0)
    parser.add_argument("-c", "--colour", help="colour/black_white", default=0)
    args = parser.parse_args()

    if args.colour == '1': colour_range = -100
    else: colour_range = 100

    dbg = False
    res = 0.05
    map_width = 657
    map_height = 731

    origin_x = 6.3
    origin_y = 6.55
    map_origin_occu = origin_y/0.05 + (origin_x/0.05)*map_width #number of cells to origin in occu
    map_origin_mat = (origin_x/0.05, origin_y/0.05)
    print "Map origin: [%s] in occu map vector" , map_origin_occu

    # Publish OccupancyGrid.
    pub_g = rospy.Publisher('/trajectory_behaviours/grid', GridCells, latch=True, queue_size=0)
    pub_o = rospy.Publisher('/trajectory_behaviours/occu', OccupancyGrid, latch=True, queue_size=0)

    rospy.init_node('visualise_qualitative_predictions', anonymous=True)

    #1. get the spatial relation distance values from config.ini
    (data_dir, config_path, params, date) = util.get_qsr_config()
    (soma_map, soma_config) = util.get_map_config(config_path)

    #Calculate a mask for each QSR (circle or donut shaped)
    qsrs = qsr_param_masks(params, dbg, colour_range, res)

    #2 Set up a grid to publish objets as binary and occupancygrid as probabilities.
    vg = visualise_grid()

    #3. get object positions - to apply the qsrs to.
    o = o_utils.get_objects_from_soma(soma_map, soma_config)
    vg.add_objects_to_grid(o)

    #print "test object:", vg.test_object, "\n", vg.test_point
    #vg.occupancygrid.data[ xy_to_occu(vg.test_point.x,vg.test_point.y)] = -100

    #print "(0, 0) =", xy_to_occu(0,0)
    #vg.occupancygrid.data[ xy_to_occu(0,0)] = 50

    #Add a test row of white in rvis:
    #for i in xrange(0, len(vg.occupancygrid.data)):
        #if i>657:continue
        #vg.occupancygrid.data[i]=1

    mat = vg.occu_to_matrix()

    #5. get Activity_Graphs info to plot:
    file_ = os.path.join(data_dir + 'TESTING/cluster_AGs.p')
    cluster_AGs = pickle.load(open(file_, "r"))

    add_masks = {}
    for i, list_of_AGs in cluster_AGs.items():
        if i != '18': continue
        print "cluster: %s. Number of graphlets: %s" % (i, len(list_of_AGs))
        for cnt, ag in enumerate(list_of_AGs):
            print "\ngraphlet: ", cnt

            print "number of objects= ", len(ag.object_nodes)
            print "number of spatials= ", len(ag.spatial_nodes)
            print "number of temporals= ", len(ag.temporal_nodes)
            for object in ag.object_nodes:
                if object['obj_type'] in o.all_objects.keys():
                    #print "  map coords:", o.all_objects[object['obj_type']]
                    key = (object['obj_type'], o.all_objects[object['obj_type']])
                    if key not in add_masks: add_masks[key] = []
                    for spatial in object.neighbors():
                        add_masks[key].append(spatial['name'])


    #pos  = xy_to_mat(vg.test_point.x, vg.test_point.y)
    #print "test object pos = ", pos
    #mat[pos] = -100
    #mat = addAtPos(mat, qsr_mask, pos, vis=dbg)
    #print add_masks.keys()

    print ""
    for cnt, ((object, map_xyz), list_of_masks) in enumerate(add_masks.items()):
        #print cnt, ((object, map_xyz), list_of_masks)
        #if cnt > 1 : continue

        for num_of_temps, qsr in enumerate(list_of_masks):
            if num_of_temps != 0: continue
            pos = xy_to_mat(map_xyz[0], map_xyz[1])
            print "Applying %s mask to %s: (%s)" %(qsr, object, pos)
            mat = o_utils.addAtPos(mat, qsrs.binary_masks[qsr], pos, add=True, vis=False)


    #print "test object pos = ", pos
    #mat = o_utils.addAtPos(mat, qsrs.binary_masks["touch"], pos, vis=dbg)
    #print mat[pos[0]-5:pos[0]+5, pos[1]-5:pos[1]+5]
    #print "origin:", mat[131,125]

    #Test final row in Mat is actually the final row in RViz
    #mat[-1] = [1]*657

    #Normalise the colours in mat. So they are colourful :)
    max_ = mat.max()
    #normalised_mat = ((max_ - mat) /max_)*100
    normalised_mat = (mat/float(max_))*-100
    #pos = (168, 92)
    #print normalised_mat[pos[0]-5:pos[0]+5, pos[1]-5:pos[1]+5]
    vg.matrix_to_occu(normalised_mat)

    vg.pub_grid(pub_g)
    vg.pub_occu(pub_o)

    rospy.spin()
