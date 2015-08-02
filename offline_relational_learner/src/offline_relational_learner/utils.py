#!/usr/bin/env python

"""utils.py: Utility functions for the visualisation of qualitative predictions"""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"
import os, sys
import rospy
import numpy as np
import time

import relational_learner.obtain_trajectories as ot


def get_objects_from_soma(soma_map, soma_config):
    print "\nquery = "
    query = {"map": soma_map,
             "config": soma_config}
    o = ot.query_objects(query)
    print "number of objects in soma: %s" % len(o.all_objects)
    return o


#Split into 2 functions?

#1. Remove a smaller mask from the center of a mask  - to make QSR masks
#2. Add a mask into  a larger mask as a certain position (pos)  -  to apply QSR masks

def addAtPos(mat1, mat2, xycoor, add=True, vis=False, first=False):
    """ - mat1  : matrix to be added to
        - mat2  : add this matrix to mat1 """

    if vis:
        print "mat shape", np.shape(mat1)
        print "mask shape", np.shape(mat2)
    size_x, size_y = np.shape(mat2)
    big_x, big_y = np.shape(mat1)

    if add:  #This is the case where you add a qsr_mask to the full occupencygrid
        adjust_x = int(size_x*0.5)
        adjust_y = int(size_y*0.5)
    else:   #this is the case where you remove the center from a donut-like qsr mask
        adjust_x, adjust_y = (big_x - size_x)/2, (big_y - size_y)/2

    coor_x, coor_y = xycoor
    end_x, end_y   = (coor_x + size_x), (coor_y + size_y)

    #Adjust for xycoor to be in the center of mat2
    if vis:
        print "adjust between mat1 and mat2: ", (adjust_x, adjust_y)
        print "ADD=", add
    if add:
        a, b = coor_x - adjust_x, end_x - adjust_x
        c, d = coor_y - adjust_y, end_y - adjust_y
    else:
        a, b = coor_x + adjust_x, end_x + adjust_x
        c, d = coor_y + adjust_y, end_y + adjust_y

    if vis:
        print "\nsize of mask:", (size_x, size_y)
        print "orig pos:", (coor_x,end_x), (coor_y,end_y)
        print "adjusted:", (a, b), (c, d)
        #print mat1[a:b, c:d]

    if a < 0:
        if vis: print "a"
        pop_cols= abs(a)
        a += pop_cols
        #print pop_cols, a
        mat2 = mat2[pop_cols:,:]

    if b > mat1.shape[0]:
        if vis: print "b"
        too_many_rows = b - mat1.shape[0]
        b -= too_many_rows
        mat2 = mat2[:-too_many_rows,:]

    if c < 0:
        if vis: print "c"
        pop_rows = abs(c)
        c += pop_rows
        print pop_rows, c
        mat2 = mat2[:,pop_rows:]

    if d > mat1.shape[1]:
        if vis: print "d"
        too_many_cols = d - mat1.shape[1]
        d -= too_many_cols
        mat2 = mat2[:,:-too_many_cols]

    if vis:
        print mat1[a:b, c:d]
        print mat2
    if len(mat2) !=0:
        if add:
            mat1[a:b, c:d] = mat1[a:b, c:d] + mat2
        else:
            if first:
                #Invert mat2. and replace mat1 with it
                mat2 = np.where(mat2 == 100, 0, 100)
                mat1[a:b, c:d] = mat2  #creates hole where smaller qsr mask is
            else:
                mat1[a:b, c:d] = mat1[a:b, c:d] - mat2
        if vis: print mat1[a:b, c:d]
    else:
        print "object+qsr tottally off the map frame"
    return mat1
