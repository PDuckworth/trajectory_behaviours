#!/usr/bin/env python

import rospy
import sys, math
import numpy as np
import cPickle as pickle
import matplotlib.pyplot as plt

class Trajectories_Heatmap(object):
    def __init__(self, pickle_file=None, data=None, vis=False):
        if pickle_file:
            with open(pickle_file, "rt") as f:
                self.data = pickle.load(f)
        elif data:
            self.data = data
        else:
            raise ValueError("both parameters cannot be empty")


    def get_binned_data(self):
        self.original_data = self.data
        self.data = {"x":[], "y":[]}

        delta = 4
        velocities=[]
        velocities_non_zero=[]

        for cnt, (x_, y_) in enumerate(zip(self.original_data["x"], \
                                           self.original_data["y"])):
            if cnt == 0:
                x = int(x_ / 0.05)
                y = int(y_ / 0.05)
            else:
                new_x = int(x_ / 0.05)
                new_y = int(y_ / 0.05)
                velocity = math.sqrt((new_x-x)**2 + (new_y-y)**2 )
                velocities.append(velocity)
                if velocity>0: velocities_non_zero.append(velocity) 
                if velocity < delta: continue

                ##Only allow each trajectory to contribute once to a bin. NOT GOOD.
                #temp = len(valid_uuid_poses)
                #valid_uuid_poses.add((x,y))
                #if len(valid_uuid_poses) == temp: continue

                x=new_x
                y=new_y
                self.data["x"].append(x)
                self.data["y"].append(y)

                #print "\n", cnt
                #print "(x, y) = ", new_x, new_y
                #print "velocity = ", velocity

        rospy.loginfo("filtering on velocity: %s to %s " %  (len(self.original_data["x"]), len(self.data["x"]))) 

        print "velocities = ", np.mean(velocities), np.median(velocities), np.max(velocities), np.std(velocities)
        print "nonzero = ", np.mean(velocities_non_zero), np.median(velocities_non_zero), np.max(non_zeros_vel), np.std(velocities_non_zero)


    def run(self):
        heatmap, xedges, yedges = self.make_heatmap(xs=self.data["x"], ys=self.data["y"])
        print "# heatmap bins:", heatmap.shape, heatmap.shape[0]*heatmap.shape[1]
        print "# votes in heatmap:", heatmap.sum()
        hflat = heatmap.flatten()
        hflat = hflat[hflat != 0]
        hmean = np.mean(heatmap)
        hmedian = np.median(heatmap)
        hstd = np.std(heatmap)
        hfmean = np.mean(hflat)
        hfmedian = np.median(hflat)
        hfstd = np.std(hflat)
        print "-heatmap\nmean:", hmean, "median:", hmedian, "std:", hstd
        print "-heatmap flat and without zeros\nmean:", hfmean, "median:", hfmedian, "std:", hfstd

        n = 2
        thres = hfmean + n*hfstd
        print "thres(%d*std): %f" %(n, thres)
        print heatmap[heatmap > thres]
        heatmap[heatmap > hfmean+2*hfstd] = 0.0

        self.plot_heatmap(heatmap=heatmap, xedges=xedges, yedges=yedges)


    def make_heatmap(self, xs, ys):
        xmin = min(xs)
        xmax = max(xs)
        ymin = min(ys)
        ymax = max(ys)
        xbins = xmax - xmin + 1
        ybins = ymax - ymin + 1
        print "xmin:", xmin, "xmax:", xmax, "ymin:", ymin, "ymax:", ymax, "xbins:", xbins, "ybins:", ybins
        heatmap, xedges, yedges = np.histogram2d(xs, ys, bins=[xbins, ybins], range= [[xmin, xmax],[ymin, ymax]])
        return heatmap, xedges, yedges


    def plot_heatmap(self, heatmap, xedges=None, yedges=None):
        if xedges is not None and yedges is not None:
            extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
            plt.imshow(heatmap.T, extent=extent, origin = 'lower')
        else:
            plt.imshow(heatmap.T, origin = 'lower')
        plt.show()

if __name__ == '__main__':
    file_name = "/home/strands/STRANDS/trajectory_dump/t_dict_text.p"
    th = Trajectories_Heatmap(pickle_file=file_name)
    th.get_binned_data()
    th.run()

