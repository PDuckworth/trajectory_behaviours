# -*- coding: utf-8 -*-
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import copy
import subprocess
import pydot

class Markov_Chain(object):
    def __init__(self, label_name, first=None, start=None):
        self.g = nx.DiGraph(label=label_name)

        if first:
            self.g.add_node('start', population=1)
            self.g.add_node(first, population=0)
            self.g.add_edge('start', first, weight=1., population=1)
        elif start:
            self.g.add_node('start', population=0)


    def handle_edge(self, start=None, end=None, x=None, y=None, new_x=None, new_y=None):
        """pass start-end nodes as string, or four coordinate values as ints"""

        if start is None and end is None:
            start = "(%s, %s)" % (x, y)
            end = "(%s, %s)" % (new_x, new_y)
        else:

            if x is None and y is None:
                start_node = "%s" % (start.replace(")", "").replace("(", ""))
                pose = start_node.split(",")
                (x, y) = np.array([float(pose[0]), float(pose[1])])
            if new_x is None and new_y is None:
                end_node = "%s" % (start.replace(")", "").replace("(", ""))
                pose = end_node.split(",")
                (new_x, new_y) = np.array([float(pose[0]), float(pose[1])])

        # check if start node exists and then update, else create a default one
        if start in self.g.nodes():
            self.g.node[start]['population'] += 1
        else:
            self.g.add_node(start, population=1)
        
        # if end node does not exist make a new default one
        if end not in self.g.nodes():
            self.g.add_node(end, population=0)
        
        # check if edge exists and take according action
        e = (start, end)
        if e in self.g.edges():
            self.g.edge[start][end]['population'] += 1
        else:
            self.g.add_edge(start, end, weight=-1, population=1)
        
        # update weights
        for k in self.g.edge[start]:
            w = float(self.g.edge[start][k]['population']) / self.g.node[start]['population']
            self.g.edge[start][k]['weight'] = w

    def display_and_save(self, layout='dot', view=False, save=True, path='data'):
        if layout == 'dot':
            self.display_dot(layout, view, save, path)
        elif layout == 'nx':
            self.display_nx(view, save, path)


    def display_dot(self, layout='dot', view=False, save=True, path='data'): # showing is saving also
        g = copy.deepcopy(self.g)
        edges = g.edges(data=True)
        for e in edges:
            e[2]['label'] = np.round(e[2]['weight'], 2)
        fname = path + '/' + str(self.g.graph['label']) + '.dot'
        nx.write_dot(g, fname)
        if save:
            fname2 = path + '/' + str(self.g.graph['label']) + '.png'
            subprocess.call(['neato', '-Tpng', fname, '-o', fname2])
        if view:
            subprocess.call(['eog', fname2])


    def display_nx(self, view=False, save=True, path='data'):
        pos = {}
        for node in self.g.nodes():
            node_new = node.replace(")", "").replace("(", "")
            label = "%s" % node_new
            pose = label.split(",")
            pos[node] = np.array([int(pose[0]), int(pose[1])])

        nx.draw_networkx(self.g, pos=pos, with_labels=False)

        #specify edge labels explicitly use round precision of 2 decimals
        pop = 0
        for u,v,d in self.g.edges(data=True):
            pop+=d["population"]
            #print ">> ", u, v, d
        print "total number of nodes in markov chain = %s" % pop

        edge_labels = dict([((u,v,), np.round(d['weight'],2)) for u,v,d in self.g.edges(data=True)])
        #check edges labels and probabilities:
        #for i, j in edge_labels.items():
        #    print i, j

        nx.draw_networkx_edges(self.g, pos, arrows=True)
        nx.draw_networkx_edge_labels(self.g, pos, edge_labels=edge_labels, label_pos=0.3)
        
        plt.text(0.99, 0.97, self.g.graph['label'], horizontalalignment='right', transform=plt.gca().transAxes)
        if save:
            fname = path + '/' + self.g.graph['label'] + '.png'
            plt.savefig(fname)
            fname = path + '/' + self.g.graph['label'] + '.gml'
            nx.write_gml(self.g, fname)
        if view:
            plt.show()




if __name__=='__main__':
    print 'hi'
       
    mc = Markov_Chain('trajectories')
    mc.handle_edge(start = '(1, 2)', end = '(0, 2)')
    mc.handle_edge(start = '(1, 2)', end = '(0, 3)')
    mc.handle_edge(start = '(1, 2)', end = '(0, 2)')
    mc.handle_edge(start = '(0, 2)', end = '(1, 2)')

    mc.handle_edge(start = '(1, 2)', end = '(1, 1)')

    mc.handle_edge(start = '(2, -1)', end = '(2, 0)')
    mc.handle_edge(start = '(2, 2)', end = '(1,2)')
    mc.handle_edge(start = '(2, 2)', end = '(-1, -2)')
    mc.handle_edge(start = '(-2, -2)', end = '(-1, -1)')

    p='/home/strands/catkin_ws/src/trajectory_behaviours/relational_learner/docs'
    mc.display_and_save(layout='nx', view=True, path=p)
         
    print 'bye'
