#!/usr/bin/env python

"""learningArea.py: File with Learing class."""

__author__      = "Paul Duckworth"
__copyright__   = "Copyright 2015, University of Leeds"

import os, sys
import rospy
import datetime, time
import math
import cPickle as pickle
import numpy as np
import pymongo
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from sklearn import mixture
from scipy.spatial.distance import pdist, cdist, euclidean
#from sklearn.metrics import pairwise_distances
from sklearn import metrics
from sklearn.cluster import KMeans, AffinityPropagation
from sklearn.feature_extraction.text import TfidfVectorizer
#from sklearn.cluster import AgglomerativeClustering
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler

from tf.transformations import euler_from_quaternion
from mongodb_store.message_store import MessageStoreProxy
from soma_geospatial_store.geospatial_store import *

from relational_learner.msg import *
from relational_learner.Activity_Graph import Activity_Graph
from time_analysis.cyclic_processes import *
from human_trajectory.msg import Trajectories

class RegionKnowledgeImporter(object):
    def __init__(self):
        rospy.loginfo("Connecting to mongodb...")
        self._client = pymongo.MongoClient(rospy.get_param("mongodb_host"),
                                           rospy.get_param("mongodb_port"))
        self._db = "message_store"
        self._collection = "region_knowledge"
        self._store_client = MessageStoreProxy(collection=self._collection)

    def find(self, query_json):
        return self._client[self._db][self._collection].find(query_json)


class Learning():
    '''
    Unsupervised Learning Class:
    Accepts a feature space, where rows are instances, and columns are features.
    '''

    def __init__(self, f_space=None, roi="", vis=False, load_from_file=None):

        if load_from_file is 'mongodb':
            self.roi = roi
            self._client = pymongo.MongoClient(
                rospy.get_param("mongodb_host", "localhost"),
                rospy.get_param("mongodb_port", 62345))
            self._retrieve_logs()

        elif load_from_file is not None and load_from_file != "":
            self.load(load_from_file)
        else:
            (self.code_book_hashes, self.code_book, \
                self.feature_space) = f_space
            self.methods = {}
            self.roi = roi
            self.visualise = vis
            self.roi_knowledge = {}
            self.roi_temp_list = {}
            self.roi_temp_know = {}
            self.X_test = {}

    def save(self, dir=None, mongodb=False, msg_store="spatial_qsr_models"):

        if mongodb:
            print("Uploading to Mongo... %s" % self.roi)
            self._store_client =  MessageStoreProxy(collection=\
                            msg_store)
            spatial_model_msg = spatialModelMsg()
            spatial_model_msg.soma_roi_id = str(self.roi)
            spatial_model_msg.methods = pickle.dumps(self.methods)
            spatial_model_msg.code_book = self.code_book_hashes
            for (ghash, graph) in self.code_book.items():
                tuple = (ghash, graph)
                spatial_model_msg.graphlet_book.append(pickle.dumps(tuple))

            self._store_client.update(message=spatial_model_msg, \
                message_query={"soma_roi_id" : str(self.roi)}, \
                upsert=True)

        else:
            print("Saving to file...roi %s" % self.roi)
            filename = os.path.join(dir, 'roi_' + self.roi + '_smartThing.p')

            foo = { "ROI": self.roi, "feature_space": self.feature_space, \
                    "code_book_hashes": self.code_book_hashes, "code_book": self.code_book, \
                    "learning_methods": self.methods, \
                    "X_test": self.X_test, "show_clstrs_in_rviz": self.cluster_trajs}

            print(filename)
            with open(filename, "wb") as f:
                pickle.dump(foo, f)
            print("success")

    def load(self, filename):
        print("Loading Learning from", filename)

        if filename== "mongo":
            print "Not implemented loading from mongo."
            sys.exit(1)

        try:
            with open(filename, "rb") as f:
                foo = pickle.load(f)
            self.roi = foo["ROI"]
            self.methods = foo["learning_methods"]
            self.code_book_hashes = foo["code_book_hashes"]
            self.code_book = foo["code_book"]
            self.feature_space = foo["feature_space"]
            self.X_test = foo["X_test"]
            self.cluster_trajs = foo["show_clstrs_in_rviz"]
            self.flag = True
            print "Loaded: " + repr(self.methods.keys())
            print("success")

        except:
            print "Loading of learnt model failed. Cannot test novelty)."
            self.flag = False


    def _retrieve_logs(self):
        query = {"soma_roi_id":str(self.roi)}
        logs = self._client.message_store.spatial_qsr_models.find(query)

        cnt=0
        for cnt, log in enumerate(logs):
            self.roi = log['soma_roi_id']
            self.methods = pickle.loads(str(log['methods']))
            self.code_book = log['code_book']
            self.code_book = {}
            for i in log['graphlet_book']:
                tuple = pickle.loads(str(i))
                self.code_book[tuple[0]] = tuple[1]
            self.methods = pickle.loads(str(log['methods']))

        if cnt != 1:
            self.flag = False
            print("%s objects in this region returned" % cnt)
        else: self.flag = True

    def pca_investigate_variables(self, apply=False):
        """
        1. Sort the eigenvalues, and select the eigenvalues with the highest.
        2. Combine the eigenvectors to form the new subspace. Matrix W.
        3. transform our samples  y= W^T * x
        4. assert transformed.shape == (k,40). where k is the reduced subspace dimension.
        """

        pca = PCA(n_components=0.99)
        #pca = PCA(n_components=3)

        print "Sample space shape >>", self.data.shape
        transf = pca.fit_transform(self.data)
        if apply:
            self.data = transf

        print ">>>pca ", pca
        print "Transform the samples onto the new subspace: ", transf.shape
        #print "> comps = ", pca.components_
        print "> var explained by top 10 components:", pca.explained_variance_ratio_[:10]

        #get_relevant_variable()
        num_features = len(pca.components_[0])
        variable_scores = [0]*num_features

        for cnt, component in enumerate(pca.components_):
            """
            For each principle component, multiply each variable by the variance the component
            explains, to obtain a score for each variable in the component.
            """
            for i, var in enumerate(component):
                variable_scores[i] += abs(var*pca.explained_variance_ratio_[cnt])

        if apply:
            pca_dict = {}
            pca_test_set = pca.transform(self.X_test.values())

            for cnt, uuid in enumerate(self.X_test.keys()):
                pca_dict[uuid] = pca_test_set[cnt]
            self.X_test = pca_dict

            self.methods["pca"] = pca

        #print ">> Weighting of each variable: \n", variable_scores, "\n"
        if self.visualise:
            plt.plot(transf, 'o', markersize=5, color='blue', alpha=0.5, label='all data')

            #plt.plot(sklearn_transf[20:40,0], sklearn_transf[20:40,1],\
            #   '^', markersize=7, color='red', alpha=0.5, label='class2')

            plt.xlabel('x_values')
            plt.ylabel('y_values')
            plt.xlim([-4,100])
            plt.ylim([-4,4])
            plt.legend()
            plt.title('Transformed samples with class labels from sklearn.PCA()')

            plt.show()
        return pca, variable_scores



    def pca_graphlets(self, pca, variable_scores, top=0.1):
        """
        visually inspect the most/least disctiminating graphlet features
        """
        max_score = max(variable_scores)
        feature1 = variable_scores.index(max_score)
        print "max score = %s" % max_score
        ghash = self.code_book_hashes[feature1]
        print ">> best feature: ", ghash, "\n", self.code_book[ghash].graph
        return


    def split_data_on_test_set(self, scale=False, test_set=None):
        rospy.loginfo('Split out Test set...')
        if scale:
            for k, v in self.feature_space.items():
                s = np.sum(v)
                foo = [i/float(s) for i in v]
                self.feature_space[k] = foo

        if test_set is None:
            self.data = np.array(self.feature_space.values())
            self.X_uuids = self.feature_space.keys()
        else:
            self.X_uuids, data_ = [], []

            for uuid, feature_vec in self.feature_space.items():
                if uuid in test_set:
                    self.X_test[uuid] = feature_vec
                else:
                    self.X_uuids.append(uuid)
                    data_.append(feature_vec)
            self.data = np.array(data_)


    def kmeans(self, k=None):
        n_samples, n_features = self.data.shape

        # determine your range of K
        if k == None:
            k_range = range(5,20)
        else:
            k_range = [k]

        # Fit the kmeans model for each n_clusters = k
        k_means_var = [KMeans(n_clusters=k, init='k-means++').fit(self.data) for k in k_range]
        # Pull out the cluster centeres for each model
        centroids = [X.cluster_centers_ for X in k_means_var]

        # Total within-cluster sum of squares (variance or inertia)
        inertia_ = [X.inertia_ for X in k_means_var]
        #print inertia_

        # silhouette scores of each model
        sil_scores = [metrics.silhouette_score(self.data, X.labels_, metric='sqeuclidean')
                for X in k_means_var]

        # Calculate the Euclidean distance from each point to each cluster center
        k_euclid = [cdist(self.data, cent, 'euclidean') for cent in centroids]
        dist = [np.min(ke, axis=1) for ke in k_euclid]

        # Keep the index of which cluster is clostest to each example
        cluster_comp = [np.argmin(ke, axis=1) for ke in k_euclid]

        # The total sum of squares
        tss = sum(pdist(self.data)**2 / n_samples)
        #print "The total sum of squares: %0.3f" % tss
        # Total within-cluster sum of squares (variance)
        wcss = [sum(d**2) for d in dist]
        # The between-cluster sum of squares
        print "Between-cluster sum of squares: %s" % (tss - wcss)
        print "Percentage of Variance explained by clustering: \n%s" % (((tss - wcss)/ tss)*100)

        print "\nRegion: %s" % self.roi
        for cnt, i in enumerate(inertia_):
            print "k: %s. inertia: %0.2f. Penalty: %0.2f. silhouette: %0.3f." \
                % (k_range[cnt], i, i*k_range[cnt], sil_scores[cnt])

        self.methods["kmeans"] = k_means_var[np.argmax(sil_scores)]
        print "k = %d has maximum silhouette score" % (len(self.methods["kmeans"].cluster_centers_))

    def cluster_radius(self, method=None):
        """ Analyse the trajectories which belong to each learnt cluster"""
        n_samples, n_features = self.data.shape
        #print "sum of inertias = ", self.estimator.inertia_
        #print "CLUSTER CENTERS = ", estimator.cluster_centers_
        #print "datapoint labels = ", self.estimator.labels_

        if method == "affinityprop":
            estimator = self.methods["affinityprop"]
        else:
            estimator = self.methods["kmeans"]

        cluster_radius, cluster_composition = {}, {}
        for cnt, (uuid, label, sample) in enumerate(zip(self.X_uuids, \
                                        estimator.labels_, self.data)):
            if method == "kmeans":
                dst = euclidean(sample, estimator.cluster_centers_[label])

            elif method == "affinityprop":
                dst = estimator.affinity_matrix_[estimator.cluster_centers_indices_[label]][cnt]

            label = str(label)
            if label not in cluster_composition:
                cluster_composition[label] = [uuid]
                cluster_radius[label] = [dst]
            else:
                cluster_composition[label].append(uuid)
                cluster_radius[label].append(dst)

        #Calculate the mean distance to the cluster center
        means, std = {}, {}
        for label in cluster_radius:
            means[label] = np.mean(cluster_radius[label])
            std[label] = np.std(cluster_radius[label])

        #Keep closest trajectories to cluster center for visualisation
        filtered_composition = {}
        for cluster_label, list_of_uuids in cluster_composition.items():
            print "Cluster %s has %s datapoints. Mean dst/sim to center = %0.3f with std = %0.3f" \
            % (cluster_label, len(list_of_uuids), means[cluster_label], std[cluster_label])

            dst_uuid = zip(cluster_radius[cluster_label], list_of_uuids)
            dst_uuid.sort()
            dst_sorted = [uuid for dst, uuid in dst_uuid]
            #Still works with AP because it stores `cosine distances`
            filtered_composition[str(cluster_label)] = dst_sorted[:30]

        if self.visualise:
            self.cluster_trajs = cluster_composition
            self.cluster_trajs_filtered = filtered_composition

        #self.methods["kmeans_composition"] = cluster_composition
        #self.methods["kmeans_composition"] = filtered_composition
        estimator.cluster_dist_means = means
        estimator.cluster_dist_std = std
        self.methods[method] = estimator


    def tf_idf_cosine_similarity_matrix(self):
        data = self.data
        (N, f) = data.shape  #Number of documents, and number of features
        print "nuber of documents = %s, number of features = %s " %(N, f)
        print data[0]

        idf_scores=[]
        feature_freq = (data != 0).sum(0)  #document_frequency
        #len_of_histograms = data.sum(1)

        for cnt, i in enumerate(feature_freq):
            idf_scores.append(math.log((N /float(i)))) if i>0 else idf_scores.append(0)
        print "feature frequency: %s" %feature_freq

        tf_idf_scores = []
        for histogram in data:
            foo = []
            for cnt, i in enumerate(histogram):
                bar = 1+math.log(i) if i>0 else 0
                foo.append(bar*idf_scores[cnt])
                #foo.append(1+math.log(i)) if i>0 else foo.append(0)
            tf_idf_scores.append(foo)

        tf_idf_scores = np.array(tf_idf_scores)

        #Length normalise the vectors to lie on the unit hypersphere
        #norms = np.linalg.norm(tf_idf_scores, axis=1)
        #len_normed_tf_idf = [tf_idf_scores[i] / float(c[i]) for i in xrange(len(norms))]

        #Calculates the 'cosine distance' which is 1-cosine similarities
        self.sim_matrix = metrics.pairwise_distances(tf_idf_scores, metric='cosine', n_jobs=4)
        #self.sim_matrix = metrics.pairwise.cosine_distances(tf_idf_scores)


    def AffinityPropagation(self):
        self.data = self.sim_matrix

        estimator = AffinityPropagation(affinity='precomputed').fit(self.data)
        n_clusters_ = len(estimator.cluster_centers_indices_)
        print('Estimated number of clusters: %d' % n_clusters_)
        print("Silhouette Coefficient: %0.3f"
              % metrics.silhouette_score(self.data, estimator.labels_, metric='sqeuclidean'))

        foo = [self.data[i] for i in estimator.cluster_centers_indices_]
        estimator.cluster_centers_ = foo
        self.methods["affinityprop"] = estimator

    def time_analysis(self, time_points, plot=False, interval=1800):
        """Number of seconds in a day = 86400"""

        first_day = int(min(time_points)/86400)

        dyn_cl = dynamic_clusters()
        for t in time_points:
            day = int(t/86400)-first_day+1
            #print day
            time_in_day = t%86400   #in seconds
            dyn_cl.add_element(day,time_in_day)

        timestamps_vec = time_wrap(time_points)[0]
        fitting = activity_time(timestamps_vec, interval=interval)

        ## Debug the Online Temporal Plot:
        #print "Saving Timepoints as a test:"
        #pickle.dump(timestamps_vec, open('/home/strands/TIME_TEST.p', "w"))

        self.methods["time_dyn_clst"] = dyn_cl
        self.methods["time_fitting"] = fitting
        if plot: self.temporal_plot(data=timestamps_vec, vis=False)
        rospy.loginfo('Done\n')

    def temporal_plot(self, data=[], plot_interval=900.0, period=86400, \
                        vis=False):
        pc = []
        pf = []
        #query model at these points to generate graph:
        timestamps = np.arange(0,period,plot_interval)
        for v in timestamps:
            pc.append(self.methods["time_dyn_clst"].query_clusters(v))
            pf.append(self.methods["time_fitting"].query_model(v))

        plot_vec = [t/3600.0 for t in timestamps]
        fig = plt.figure()
        t_ax = fig.add_subplot(111)
        width=0.01
        plt.bar(plot_vec,pc,width, color='r', edgecolor='r',\
              label='dynamic clustering')
        plt.plot(plot_vec,pf,label='GMM fitting')
        plt.xlim([0,24])
        t_ax.set_xticks(np.arange(0,24))

        plt.xlabel('time of day')
        plt.ylabel('probability')
        plt.legend()
        filename='/tmp/temporal_plot_%s.jpg' % self.roi
        plt.savefig(filename, bbox_inches='tight', dpi=100)

















def region_knowledge(map, config, interval=3600.0, period = 86400.0,\
                    sampling_rate=10, plot=False):

    """Returns the ROIs the robot has montitor at each logged robot pose"""
    t0 = time.time()
    n_bins = int(period/interval)

    ##Get info stored in Mongodb Region Knowledge Store
    ks = RegionKnowledgeImporter()
    roi_knowledge = {}
    roi_temp_list = {}

    query = {"soma_roi_id"  : {"$exists": "true"},
            "roi_knowledge" : {"$exists": "true"}}
    for region in ks.find(query):
        roi_knowledge[str(region["soma_roi_id"])] = int(region["roi_knowledge"])
        roi_temp_list[str(region["soma_roi_id"])] = region["roi_knowledge_hourly"]

    print "existing knowledge = ", roi_knowledge
    print "existing hourly knowledge = ", roi_temp_list

    ##Query the Robot Poses from roslog
    gs = GeoSpatialStoreProxy('geospatial_store','soma')
    ms = GeoSpatialStoreProxy('message_store','soma_roi')
    roslog = GeoSpatialStoreProxy('roslog','robot_pose')

    ###Only query previous 24 hours worth of robot_poses
    now = datetime.datetime.today()
    delta = datetime.timedelta(hours=24)
    query_date = now-delta
    query = {"_id": {"$exists": "true"}, "_meta.inserted_at": {"$gte":query_date}}

    print "date = ", now
    print "querying date = ", query_date
    print "sampling rate =", sampling_rate

    cnt=0
    ##Loop through the robot poses for the day
    for cnt, p in enumerate(roslog.find(query)):
        if cnt % sampling_rate != 0: continue   #Take 1/10 of the roslog poses
        timepoint = cnt/sampling_rate

        #print p
        pose = p['position']
        ro, pi, yaw = euler_from_quaternion([0, 0, \
                p['orientation']['z'], p['orientation']['w'] ])

        inserted_at = p['_meta']['inserted_at']
        hour = inserted_at.time().hour

        coords = robot_view_cone(pose['x'], pose['y'], yaw)
        lnglat = []
        for pt in coords:
            lnglat.append(gs.coords_to_lnglat(pt[0], pt[1]))
        #add first points again to make it a complete polygon
        lnglat.append(gs.coords_to_lnglat(coords[0][0], coords[0][1]))

        for i in gs.observed_roi(lnglat, map, config):
            region = str(i['soma_roi_id'])

            #Region Knowledge
            if region in roi_knowledge:
                roi_knowledge[region]+=1
            else:
                roi_knowledge[region]=1

            #Region Knowledge per hour. Bin them by hour.
            if region in roi_temp_list:
                roi_temp_list[region][hour]+=1
            else:
                roi_temp_list[region]=[0]*24
                roi_temp_list[region][hour] = 1

    print "number of robot poses queried = ", cnt

    print "roi_knowledge = ", roi_knowledge
    print "roi_temporal_knowledge = ", roi_temp_list

    #update mongodb (as the roslog/robot_pose data is removed at the end of the day)
    for roi, score in roi_knowledge.items():
        region_type = gs.type_of_roi(roi, map, config)

        msg = RegionKnowledgeMsg(soma_roi_id = roi, type = str(region_type), \
                   roi_knowledge = score, roi_knowledge_hourly = roi_temp_list[roi])

        query = {"soma_roi_id" : roi}
        #print "MESSAGE = \n", msg
        #print "query = ", query
        p_id = ks._store_client.update(message=msg, message_query=query, meta={}, upsert=True)

    print "Knowledge of Regions takes: ", time.time()-t0, "  secs."
    if plot: knowledge_plot(roi_temp_list, n_bins)

    rospy.loginfo('Done')
    return roi_knowledge, roi_temp_list


def knowledge_plot(roi_temp_list, n_bins):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    z = 0
    cl = ['r', 'g', 'b', 'y']
    regions=[]
    for (roi, k) in roi_temp_list.items():
        #print k
        regions.append(roi)
        cls = [cl[z%4]]*n_bins
        ax.bar(range(n_bins),k, zs=z, zdir='y', color=cls, alpha = 0.8)
        z = z+1
    ax.set_ylabel("ROI")
    ax.set_xlabel("time")
    ax.set_zlabel("observation (secs)/area (m^2)")
    ax.set_xticks([0,3,6,9,12,15,18,21,24])
    ax.set_yticks(range(1,len(regions)+1))
    ax.set_yticklabels(regions)
    date = time.strftime("%x").replace("/","_")
    filename='/tmp/roi_knowledge__%s.jpg' % date
    plt.savefig(filename, bbox_inches='tight', dpi=100)


def robot_view_cone( Px, Py, yaw):
    """ let's call the triangle PLR, where P is the robot pose,
        L the left vertex, R the right vertex"""
    d = 4 # max monitored distance: reasonably not more than 3.5-4m
    alpha = 1 # field of view: 57 deg kinect, 58 xtion, we can use exactly 1 rad (=57.3 deg)
    Lx = Px + d * (math.cos((yaw-alpha)/2))
    Ly = Py + d * (math.cos((yaw-alpha)/2))
    Rx = Px + d * (math.cos((yaw+alpha)/2))
    Ry = Py + d * (math.cos((yaw+alpha)/2))
    return [ [Lx, Ly], [Rx, Ry], [Px, Py] ]


def plot_pca(data, k):
    ###############################################################################
    # Visualize the results on PCA-reduced data
    reduced_data = PCA(n_components=2).fit_transform(data)
    kmeans = KMeans(init='k-means++', n_clusters=k, n_init=10)
    kmeans.fit(reduced_data)

    # Step size of the mesh. Decrease to increase the quality of the VQ.
    h = .02     # point in the mesh [x_min, m_max]x[y_min, y_max].

    # Plot the decision boundary. For that, we will assign a color to each
    x_min, x_max = reduced_data[:, 0].min() + 1, reduced_data[:, 0].max() - 1
    y_min, y_max = reduced_data[:, 1].min() + 1, reduced_data[:, 1].max() - 1
    xx, yy = np.meshgrid(np.arange(x_min, x_max, h), np.arange(y_min, y_max, h))

    # Obtain labels for each point in mesh. Use last trained model.
    Z = kmeans.predict(np.c_[xx.ravel(), yy.ravel()])

    # Put the result into a color plot
    Z = Z.reshape(xx.shape)
    plt.figure(1)
    plt.clf()
    plt.imshow(Z, interpolation='nearest',
               extent=(xx.min(), xx.max(), yy.min(), yy.max()),
               cmap=plt.cm.Paired,
               aspect='auto', origin='lower')


    plt.plot(reduced_data[:, 0], reduced_data[:, 1], 'k.', markersize=2)
    centroids = kmeans.cluster_centers_         # Plot the centroids as a white X
    print centroids
    plt.scatter(centroids[:, 0], centroids[:, 1],
                marker='x', s=169, linewidths=3,
                color='w', zorder=10)
    plt.title('K-means clustering on the Resource Room/trajectories data (PCA-reduced)\n'
             'Centroids are marked with white cross')
    plt.xlim(x_min, x_max)
    plt.ylim(y_min, y_max)
    plt.xticks(())
    plt.yticks(())
    plt.show()


def test_plot(data=[], plot_interval=900.0, period=86400):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    width=0.01

    timestamps_vec = time_wrap(data)[0]
    plt.hist(timestamps_vec)
    plt.xlim([0,86400])

    plt.xlabel('time of day')
    plt.ylabel('count')
    plt.legend()
    plt.show()



if __name__ == "__main__":
    rospy.init_node('learningArea')
    data_dir = '/home/strands/STRANDS/'

    ##file_ = os.path.join(data_dir + 'AG_graphs/feature_space_roi_2_None_1_3_4__13_04_2015.p')
    #file_ = os.path.join(data_dir + 'AG_graphs/feature_space_roi_1_None_1_3_4__13_04_2015.p')
    #print file_
    #l = pickle.load(open(file_, "r"))
    #(code_book, graphlet_book, X_source_U, X_uuids) = l
    #print "# datapoints = ", len(X_uuids)#, X_uuids

    #smartThing=Learning(f_space=l)
    #smartThing.kmeans()
    #print "Learnt models for: "
    #for key in smartThing.methods:
    #    print "    ", key
    #smartThing.save(data_dir + 'learning')


    ##Check trajectory clusters
    file_ = os.path.join(data_dir + 'learning/roi_2_smartThing_filtered.p')
    print file_
    smartThing=Learning(load_from_file=file_)
    print smartThing.methods["kmeans_cluster_composition"].keys()
    kmeans_analysis(smartThing.methods["kmeans_cluster_composition"], vis=True)



    ##Check the temporal models
    #file_ = os.path.join(data_dir + 'learning/roi_2_smartThing.p')
    #print file_
    #smartThing=Learning(load_from_file=file_)

    #time_vec = smartThing.methods['temporal_list_of_uuids']
    #print len(time_vec)

    #test_plot(time_vec)
