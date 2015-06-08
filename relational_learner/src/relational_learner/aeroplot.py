#!/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt



def aeroplot_test(t_heatmap, interest_points):
    xmin = min(t_heatmap.data["x"])
    xmax = max(t_heatmap.data["x"])
    ymin = min(t_heatmap.data["y"])
    ymax = max(t_heatmap.data["y"])

    X, Y = np.meshgrid(t_heatmap.xedges, t_heatmap.yedges)   # generates a mesh grid

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

