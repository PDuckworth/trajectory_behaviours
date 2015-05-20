#!/usr/bin/env python

import sys
import math
import numpy as np
import cPickle as pickle
import matplotlib.pyplot as plt
import argparse

class Trajectories_Heatmap(object):
    def __init__(self, bin_size, pickle_file=None, data=None, verbose=False):
        self.bin_size = bin_size
        self.verbose = verbose
        self.data = None
        self.heatmap = None
        if pickle_file or data:
            self.set_data(pickle_file=pickle_file, data=data, bin_size=bin_size)
        if verbose and self.data is None:
            print("Warning: data unitialized, run set_data")


    def set_data(self, bin_size=None, pickle_file=None, data=None):
        if pickle_file is None and data is None:
            raise ValueError("both parameters cannot be empty")
        if bin_size is None:
            bin_size = self.bin_size
        if pickle_file:
            with open(pickle_file, "rt") as f:
                data = pickle.load(f)
        self.data = self.get_binned_data_and_filter_by_velocity(data, bin_size)


    def get_binned_data_and_filter_by_velocity(self, data, bin_size=0.05):
        data_ret = {"x":[], "y":[]}

        delta = 4
        velocities=[]
        velocities_non_zero=[]

        for cnt, (x_, y_) in enumerate(zip(data["x"], data["y"])):
            if cnt == 0:
                x = int(x_ / bin_size)
                y = int(y_ / bin_size)
            else:
                new_x = int(x_ / bin_size)
                new_y = int(y_ / bin_size)
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
                data_ret["x"].append(x)
                data_ret["y"].append(y)

                #print "\n", cnt
                #print "(x, y) = ", new_x, new_y
                #print "velocity = ", velocity

        if self.verbose:
            print "filtering on velocity: %s to %s " %  (len(self.original_data["x"]), len(self.data["x"]))
            print "velocities = ", np.mean(velocities), np.median(velocities), np.max(velocities), np.std(velocities)
            print "nonzero = ", np.mean(velocities_non_zero), np.median(velocities_non_zero), np.max(velocities_non_zero), np.std(velocities_non_zero)

        return data_ret


    def run(self, vis=False):
        self.compute_heatmap()
        self.filter_outliers()
        if vis:
            self.plot_heatmap()


    def filter_outliers(self, mean_or_median="mean", std_factor=2, set_value=0.0, analyze=False):
        # heatmap, xedges, yedges = self.compute_heatmap(xs=self.data["x"], ys=self.data["y"])

        hflat = self.heatmap.flatten()
        hflat = hflat[hflat != 0]
        if mean_or_median == "mean":
            hfc = np.mean(hflat)
        elif mean_or_median == "median":
            hfc = np.median(hflat)
        else:
            raise ValueError("incorrect value for mean_or_median parameter, must be 'mean' or 'median'")
        hfstd = np.std(hflat)
        thres = hfc + std_factor * hfstd
        if set_value < 0:
            set_value = thres
        self.heatmap[self.heatmap > thres] = set_value

        if analyze:
            ahmean = np.mean(self.heatmap)
            ahmedian = np.median(self.heatmap)
            ahstd = np.std(self.heatmap)
            ahfmean = np.mean(hflat)
            ahfmedian = np.median(hflat)
            ahfstd = np.std(hflat)
            if self.verbose:
                print "# heatmap bins:", self.heatmap.shape, self.heatmap.shape[0]*self.heatmap.shape[1]
                print "# votes in heatmap:", self.heatmap.sum()
                print "-heatmap\nmean:", ahmean, "median:", ahmedian, "std:", ahstd
                print "-heatmap flat and without zeros\nmean:", ahfmean, "median:", ahfmedian, "std:", ahfstd
                print "thres(%d*std): %f" %(std_factor, thres)
                print self.heatmap[self.heatmap > thres]


    def compute_heatmap(self):
        try:
            xmin = min(self.data["x"])
            xmax = max(self.data["x"])
            ymin = min(self.data["y"])
            ymax = max(self.data["y"])
        except:
            raise ValueError("data not set, use set_data method first")
        xbins = xmax - xmin + 1
        ybins = ymax - ymin + 1
        if self.verbose: print "xmin:", xmin, "xmax:", xmax, "ymin:", ymin, "ymax:", ymax, "xbins:", xbins, "ybins:", ybins
        self.heatmap, self.xedges, self.yedges = np.histogram2d(self.data["x"], self.data["y"],
                                                                bins=[xbins, ybins], range=[[xmin, xmax],[ymin, ymax]])


    def plot_heatmap(self, heatmap=None, xedges=None, yedges=None):
        if heatmap is None: heatmap = self.heatmap
        if xedges is None: xedges = self.xedges
        if yedges is None: yedges = self.yedges

        if xedges is not None and yedges is not None:
            extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
            plt.imshow(heatmap.T, extent=extent, origin = 'lower')
        else:
            plt.imshow(heatmap.T, origin='lower')
        plt.show()



if __name__ == '__main__':
    argp = argparse.ArgumentParser()
    argp.add_argument("-l", "--load", required=True, help="pickle file to load data from")
    argp.add_argument("-v", "--view", action="store_true", help="view the heatmap")
    args = argp.parse_args()

    th = Trajectories_Heatmap(bin_size=0.05, pickle_file=args.load)
    th.run(vis=args.view)
