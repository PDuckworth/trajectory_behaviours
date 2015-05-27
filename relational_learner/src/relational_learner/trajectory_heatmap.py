#!/usr/bin/env python

import sys
import math
import numpy as np
import cPickle as pickle
import matplotlib.pyplot as plt
import argparse
import convex_hull as ch

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
                x = int(x_ / float(bin_size))
                y = int(y_ / float(bin_size))
            else:
                new_x = int(x_ / float(bin_size))
                new_y = int(y_ / float(bin_size))

                #if new_y > 0: continue

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
            print "nonzero = ", np.mean(velocities_non_zero), np.median(velocities_non_zero), \
                        np.max(velocities_non_zero), np.std(velocities_non_zero)

        return data_ret


    def run(self, vis=False, with_analysis=False):
        self.compute_heatmap()
        self.filter_outliers(with_analysis=with_analysis)
        if vis:
            self.plot_heatmap()


    def filter_outliers(self, mean_or_median="mean", std_factor=2, set_value=0.0, with_analysis=False):
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
        if thres_low<0: thres_low=1
        thres_high = hfc + std_factor * hfstd

        if set_value < 0:
            set_value = thres

        if with_analysis:
            ahmean = np.mean(self.heatmap)
            ahmedian = np.median(self.heatmap)
            ahstd = np.std(self.heatmap)
            ahfmean = np.mean(hflat)
            ahfmedian = np.median(hflat)
            ahfstd = np.std(hflat)
            print "# heatmap bins:", self.heatmap.shape, self.heatmap.shape[0]*self.heatmap.shape[1]
            print "# votes in heatmap:", self.heatmap.sum()
            print "-heatmap\nmean:", ahmean, "median:", ahmedian, "std:", ahstd
            print "-heatmap flat and without zeros\nmean:", ahfmean, "median:", ahfmedian, "std:", ahfstd
            print "threshold high (%d*std): %f" %(std_factor, thres_high)
            print "following will be filtered out\n", self.heatmap[self.heatmap > thres_high]
            print "threshold low (%d*std): %f" %(std_factor, thres_low)
            print "following will be filtered out\n", self.heatmap[self.heatmap < thres_low]
        
        (xs, ys) = np.where(self.heatmap > thres_high)
        #print ">>>", (xs, ys)

        #LowPass filter. More than one trajectory in each bin
        self.heatmap[self.heatmap <= thres_low] = set_value
        #HighPass filter. Remove bins with > then a couple of std's away from the mean
        self.heatmap[self.heatmap > thres_high] = set_value

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
        print "shape of heatmap = ", self.heatmap.shape
        print "heatmap = ", self.heatmap
        #print "xedges = ", self.xedges
        #print "yedges = ", self.yedges
        print "bins = ", [xbins, ybins]
        print "range = ", [[xmin, xmax],[ymin, ymax]]


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


    def plot_polygon(self, vis=False, **kwargs):
        xmin = min(self.data["x"])
        ymin = min(self.data["y"])

        if self.heatmap.shape[0]==0:
            raise ValueError("Cannot fit polygon to trajectories. No heatmap points")

        #Keep only one point per bin (just to make sure)
        (xs, ys) = np.where(self.heatmap !=0)

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


    def aeroplot_test(self, interest_points):
        xmin = min(self.data["x"])
        xmax = max(self.data["x"])
        ymin = min(self.data["y"])
        ymax = max(self.data["y"])

        X, Y = np.meshgrid(self.xedges, self.yedges)   # generates a mesh grid

        print "here:", len(X), len(Y), len(X[0]), len(Y[0])
        print "Plotting quiver..."

        plt.xlabel('x', fontsize=16)
        plt.ylabel('y', fontsize=16)
        plt.xlim(xmin, xmax)
        plt.ylim(ymin, ymax)
        #plt.scatter(X, Y, s=1, color='#CD2305', marker='o', linewidth=0)

        for source_point in interest_points:
            print source_point
            u_pair, v_pair = add_source_point(X, Y, source_point, 5.0)

            for sink_point in interest_points:
                if source_point[0] == sink_point[0] and source_point[1] == sink_point[1]: continue

                u_sink, v_sink = add_sink_point(X, Y, sink_point, 5.0)
                u_pair+=u_sink
                v_pair+=v_sink

            plt.streamplot(X, Y, u_pair, v_pair, density=2.0, linewidth=1, arrowsize=2, arrowstyle='->')
            plt.show()




def add_source_point(X, Y, interest_point, strength=5.0):
    strength_source = strength                                    # source strength
    x_source, y_source = interest_point[0], interest_point[1]     # location of the source

    # computes the velocity field on the mesh grid
    u_source = strength_source/(2*math.pi) * (X-x_source)/((X-x_source)**2 + (Y-y_source)**2)
    v_source = strength_source/(2*math.pi) * (Y-y_source)/((X-x_source)**2 + (Y-y_source)**2)

    #plt.streamplot(X, Y, u_source, v_source, density=1, linewidth=1, arrowsize=2, arrowstyle='->')
    plt.scatter(x_source, y_source, color='r', s=80, marker='o', linewidth=0)
    return u_source, v_source


def add_sink_point(X, Y, interest_point, strength=5.0):
    strength_sink = -5.0                                           # strength of the sink
    x_sink, y_sink = interest_point[0], interest_point[1]            # location of the sink

    # computes the velocity on the mesh grid
    u_sink = strength_sink/(2*math.pi) * (X-x_sink)/((X-x_sink)**2 + (Y-y_sink)**2)
    v_sink = strength_sink/(2*math.pi) * (Y-y_sink)/((X-x_sink)**2 + (Y-y_sink)**2)

    #plt.streamplot(X, Y, u_sink, v_sink, density=2, linewidth=1, arrowsize=2, arrowstyle='->')
    plt.scatter(x_sink, y_sink, color='b', s=80, marker='o', linewidth=0)
    return u_sink, v_sink


if __name__ == '__main__':
    argp = argparse.ArgumentParser()
    argp.add_argument("-l", "--load", required=True, help="pickle file to load data from")
    argp.add_argument("-v", "--view", action="store_true", help="view the heatmap")
    args = argp.parse_args()

    th = Trajectories_Heatmap(bin_size=0.05, pickle_file=args.load)
    #th.run(vis=args.view, with_analysis=True)
    th.run(vis=False, with_analysis=True)
    th.plot_heatmap()

    interest_points = th.plot_polygon(vis=True, facecolor='green', alpha = 0.4)
    print interest_points
    #pickle.dump(th.data, open("/home/strands/convex_points.p", "w"))


    clustered_hull_pts = [[  66., -134.],  [96., -118.], [65., 5.], [27., 16.], \
        [-83., -1.], [-106., -22.], \
        [ -89., -54.], [-31., -148.]]

    th.aeroplot_test(clustered_hull_pts)