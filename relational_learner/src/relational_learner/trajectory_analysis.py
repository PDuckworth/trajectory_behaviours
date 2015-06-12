#!/usr/bin/env python

import sys
import math
import numpy as np
import cPickle as pickle
import matplotlib.pyplot as plt
import argparse
import convex_hull as ch
import mc_networkx as mc
import aeroplot as aero

class Discretise_Trajectories(object):
    def __init__(self, bin_size, filter_vel, pickle_file=None, data=None, verbose=False):
        self.bin_size = bin_size
        self.data = None
        self.velocity_delta = filter_vel
        self.verbose = verbose
        self.heatmap = None

        if pickle_file or data:
            self.set_data(pickle_file=pickle_file, data=data)
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

        self.data = self.get_binned_data_and_filter_by_velocity(data)


    def get_binned_data_and_filter_by_velocity(self, data):

        bin_size = self.bin_size
        data_ret = {"x":[], "y":[], "angles":[], "velocities":[]}

        #Create 8 circular bins, with centers: [90, 45, 0, -45, -90, -135, 180, 135]
        slices = np.linspace(22.5, 382.5, 9, True).astype(np.float) - 180
        slices.sort()

        #Rename the binned angles as per a clock face:
        binned_angles = {6:0, 5:1, 4:2, 3:3, 2:4, 1:5, 0:6, 8:6, 7:7}

        mchain = mc.Markov_Chain('trajectories')
        print "number of subject ids: ", data.keys()
        for uuid, data in data.items():
            xs, ys, angles, velocities = [], [], [], []   #valid x and y's with positive velocity

            for cnt, (x_, y_) in enumerate(zip(data["x"], data["y"])):
                hype=0
                if cnt == 0:
                    x = int(x_ / float(bin_size))
                    y = int(y_ / float(bin_size))
                    xs.append(x)
                    ys.append(y)
                    #print "\nfirst x, y = ", x, y
                else:
                    new_x = int(x_ / float(bin_size))
                    new_y = int(y_ / float(bin_size))

                    #Create the velocity and angle between two poses:
                    len_x = (new_x-x)
                    len_y = (new_y-y)
                    hype = math.hypot(len_x, len_y)
                    if hype < self.velocity_delta: continue

                    angle_degrees = math.atan2(len_x, len_y) * 180 / np.pi
                    digitized = np.digitize([angle_degrees], slices)

                    if self.verbose:
                        print "\n(x, y) = %s, %s. Velocity = %s" % (new_x, new_y, hype)
                        print "angle from previous point = %s" %(angle_degrees)
                        print "discretized bin = %s, new bin = %s" % (digitized, binned_angles[digitized[0]])

                    mchain.handle_edge(x=x, y=y, new_x=new_x, new_y=new_y)

                    x = new_x
                    y = new_y
                    xs.append(x)
                    ys.append(y)
                    velocities.append(hype)
                    angles.append(binned_angles[digitized[0]])
            if self.verbose: print "#poses ", cnt
            #remove the last point (since it doesnt have an velocity/angle)
            if len(xs) == len(velocities)+1: del xs[-1], ys[-1]

            data_ret["x"].extend(xs)
            data_ret["y"].extend(ys)
            data_ret["angles"].extend(angles)
            data_ret["velocities"].extend(velocities)

            print "len of vel= ",  len(velocities)
            print velocities

        self.markov_chain = mchain
        print "valid datapoints after filtering on velocity = %s " % (len(data_ret["angles"]))
        return data_ret

    def heatmap_run(self, vis=False, with_analysis=False):
        self.compute_heatmap()
        self.filter_outliers(with_analysis=with_analysis)
        if vis:
            self.plot_heatmap()

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
        zbins = 8

        d3_data = np.array(zip(self.data["x"], self.data["y"], self.data["angles"]))
        print "#datapoints: %s, example: %s, ie. shape: %s" % (len(d3_data), d3_data[0], d3_data.shape)


        self.heatmap, (self.xedges, self.yedges, self.zedges) = np.histogramdd(d3_data, \
                    bins=[xbins, ybins, zbins], range=[[xmin, xmax],[ymin, ymax], [0, 7]])

        #self.heatmap, self.xedges, self.yedges = np.histogram2d(self.data["x"], self.data["y"],\
        #                        bins=[xbins, ybins], range=[[xmin, xmax],[ymin, ymax]])

        if self.verbose:
            print "heatmap.shape =  ", self.heatmap.shape
            print "bin edges.size = %s, %s, %s" % (self.xedges.size, self.yedges.size, self.zedges.size)

            print "xmin:", xmin, ", xmax:", xmax, ", ymin:", ymin, ", ymax:", ymax
            print "xbins:", xbins, ", ybins:", ybins, "zbins:", zbins
            print "shape of heatmap = ", self.heatmap.shape
            print "range = ", [[xmin, xmax],[ymin, ymax], [0, 7]]
            #print "heatmap = ", self.heatmap
            #print "xedges = ", self.xedges
            #print "yedges = ", self.yedges


    def filter_outliers(self, mean_or_median="mean", std_factor=1, set_value=0.0, with_analysis=False):
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

        thres_low = hfc - std_factor * hfstd
        if thres_low<1: thres_low=1
        thres_high = hfc + std_factor * hfstd

        if set_value < 0:
            set_value = thres_low

        if with_analysis:
            ahmean = np.mean(self.heatmap)
            ahmedian = np.median(self.heatmap)
            ahstd = np.std(self.heatmap)
            ahfmean = np.mean(hflat)
            ahfmedian = np.median(hflat)
            ahfstd = np.std(hflat)
            print "# heatmap bins:", self.heatmap.shape, self.heatmap.shape[0]*self.heatmap.shape[1]*self.heatmap.shape[2]
            print "# votes in heatmap:", self.heatmap.sum()
            print "-heatmap\n  mean:", ahmean, "median:", ahmedian, "std:", ahstd
            print "-heatmap flat and without zeros\n  mean:", ahfmean, "median:", ahfmedian, "std:", ahfstd
            print "-thresholds\n  high (%d*std): %f" %(std_factor, thres_high)
            print "High pass filters out\n", self.heatmap[self.heatmap > thres_high]
            print "  low (%d*std): %f" %(std_factor, thres_low)
            print "  Low pass filters out %s bins\n" % ( len(self.heatmap[self.heatmap == 1]) )

        #(xs, ys, zs) = np.where(self.heatmap > thres_high)
        #print ">>>", (xs, ys)

        #LowPass filter. More than one trajectory in each bin
        #self.heatmap[self.heatmap <= thres_low] = set_value

        #HighPass filter. Remove bins with > then a couple of std's away from the mean
        self.heatmap[self.heatmap > thres_high] = thres_high




    def plot_heatmap(self, heatmap=None, xedges=None, yedges=None):
        if heatmap is None: heatmap = np.sum(self.heatmap, axis=2)
        if xedges is None: xedges = self.xedges
        if yedges is None: yedges = self.yedges

        if xedges is not None and yedges is not None:
            extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]

            #plt.imshow(heatmap.T, extent=extent,  \
            #            interpolation='nearest', cmap='hot')
            X,Y = np.meshgrid(xedges, yedges)
            plt.pcolor(X, Y, heatmap.T, cmap='hot')
        else:
            plt.imshow(heatmap.T, origin='lower', cmap=plt.cm.gray)
        plt.colorbar()
        plt.grid(True)
        plt.show()


    def plot_polygon(self, vis=False, **kwargs):
        xmin = min(self.data["x"])
        ymin = min(self.data["y"])

        if self.heatmap.shape[0]==0:
            raise ValueError("Cannot fit polygon to trajectories. No heatmap points")

        #Keep only one point per bin (just to make sure)
        (xs, ys, zs) = np.where(self.heatmap !=0)

        #This is currently an approximation, because points have been binned previously.
        xs = [ float(i+xmin) for i in xs]
        ys = [ float(i+ymin) for i in ys]

        xy = zip(xs, ys)
        xy = set(xy)
        points = np.array((zip(*xy)))
        #points = np.array((xs, ys))

        print "points >", points.shape
        hull_pts = ch.convex_hull(points, graphic=vis)
        print "hull_points size >>", hull_pts.shape

        ch.draw_polygon(hull_pts, **kwargs)
        if vis: plt.show()

        return hull_pts



class Trajectory_Markov_Chain(object):
    def __init__(self, bin_size, filter_vel, pickle_file=None, data=None, verbose=False):
        self.bin_size = bin_size
        self.verbose = verbose
        self.data = None
        self.heatmap = None
        self.velocity_delta = filter_vel
        if pickle_file or data:
            self.set_data(pickle_file=pickle_file, data=data, bin_size=bin_size)
        if verbose and self.data is None:
            print("Warning: data unitialized, run set_data")




if __name__ == '__main__':
    argp = argparse.ArgumentParser()
    argp.add_argument("-l", "--load", required=True, help="pickle file to load data from")
    argp.add_argument("-v", "--view", action="store_true", help="view the heatmap")
    args = argp.parse_args()

    dt = Discretise_Trajectories(bin_size=0.1, delta = 1, pickle_file=args.load, verbose=False)
    dt.heatmap_run(vis=args.view, with_analysis=True)

    p='/home/strands/STRANDS/trajectory_dump'
    th.markov_chain.display_and_save(layout='nx', view=True, path=p)

    interest_points = th.plot_polygon(vis=True, facecolor='green', alpha = 0.4)
    print interest_points
    #pickle.dump(th.data, open("/home/strands/convex_points.p", "w"))

    #Run some clustering on the 2d hull points:
    clustered_hull_pts = [[  66., -134.],  [96., -118.], [65., 5.], [27., 16.], \
        [-83., -1.], [-106., -22.], \
        [ -89., -54.], [-31., -148.]]

    #aero.aeroplot_test(th, clustered_hull_pts)