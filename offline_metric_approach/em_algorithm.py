#!/usr/bin/env python

"""Use the same trajectories as the qualitative approach, and
attempt to fit a fully metric EM learning approach"""

from __future__ import division
import os, sys, copy
import cPickle as pickle
import datetime, time
import numpy as np
import math


def load_data(max_trajs, file='/home/strands/STRANDS/TESTING/offline_UUIDS_to_include_in_metric_roi_1', vis=False):
	##Get date from pickle
	with open(file, "rb") as f:
		data = pickle.load(f)

	print "number of trajectories loaded = ", len(data)
	print "number of trajectories being used: %s" % max_trajs

	max_number_of_poses = 0

	reduced_data = {}
	for cnt, (uuid, poses) in enumerate(data.items()):
		if cnt >= max_trajs: continue

		if vis: print "%s: %s Poses: %s" % (cnt, uuid, len(poses))
		if len(poses) > max_number_of_poses: max_number_of_poses = len(poses)

		if max_traj is not None:
			reduced_data[uuid] = poses

	print "max poses: %s" % max_number_of_poses
	if max_traj is not None: data = reduced_data

	return data, max_number_of_poses


def fix_data_length(data, max_number_of_poses=None, file=None):
	if max_number_of_poses is not None:

		new_data = {}
		for uuid, poses in data.items():
			diff = max_number_of_poses - len(poses)
			new_poses = [poses[-1]]*diff
			poses.extend(new_poses)
			new_data[uuid] = poses

		if file is not None:
			pickle.dump(new_data, open(file, "wb"))
			print "saved: %s" %file
		else:
			return new_data


def discretise_data(data, bin_size):

	data_ret = {"x":[], "y":[], "angles":[], "velocities":[]}
	print "number of subject ids: ", len(data)

	discretised_data = {}
	for cnt, (uuid, poses) in enumerate(data.items()):

		xs, ys, angles, velocities = [], [], [], []   #valid x and y's with positive velocity
		p = []
		for x_, y_, z_ in poses:

			x = int(x_ / float(bin_size))
			y = int(y_ / float(bin_size))
			p.append((x, y))

		discretised_data[uuid] = p
	return discretised_data


def E_init_step(params, vis=False):
	"""
	Initialises the expectation matrix with a unimodal distribution - for each trajectory
	return: matrix of Expected values, shape (NxM) = (#of trajectories, #of motion behaviours)
	"""
	E_mat = np.zeros(params["num_of_motions"]*params["num_of_trajs"]).\
							reshape(params["num_of_trajs"], params["num_of_motions"])

	for i in xrange(0, params["num_of_trajs"]):
		random_vec = np.random.rand(params["num_of_motions"])
		dist_vec = random_vec / sum(random_vec)
		E_mat[i,:] = dist_vec

		if vis:
			print "subj i = %s out of %s " % (i, params["num_of_trajs"])
			print E_mat[i,:]
			#print random_vec, sum(random_vec)
			#print dist_vec, sum(dist_vec)
	#print E_mat.sum(axis=1)
	return E_mat


def M_step(it, E_mat, mu, data, params, vis=False):
	"""
	returns: matrix of Mean values (mu), shape (MxT) = (#of motion behaviours x #of timepoints)
	"""
	if vis: print "\nM STEP %s" % it
	num_of_motions = params["num_of_motions"]
	num_of_trajs = params["num_of_trajs"]
	num_of_timepoints = params["num_of_timepoints"]

	(uuids, all_poses) = data

	for motion in xrange(0, num_of_motions):
		if vis: print "\nmotion: ", motion
		for t in xrange(0, num_of_timepoints):
			if vis: print "\n  time: ", t

			nominator_x, nominator_y, denominator = 0, 0, 0
			if vis: print "  E_mat = %s  " % E_mat[:, motion]
			for cnt, (uuid, poses) in enumerate(zip(uuids, all_poses)):
				if cnt>=(num_of_trajs): continue

				x, y = poses[t]
				if vis: print "    subject: %s \n    E = %s, poses: %s" % (cnt, E_mat[cnt, motion], poses[t])

				nominator_x += E_mat[cnt, motion]*x
				nominator_y += E_mat[cnt, motion]*y
				denominator += E_mat[cnt, motion]
				if vis: print "    nom_x: %s, nom_y: %s, denom %s" %(E_mat[cnt, motion]*x, \
													E_mat[cnt, motion]*y, E_mat[cnt, motion])

			mu[motion, t] = (nominator_x/float(denominator), nominator_y/float(denominator))
			if vis: print "  mu (%s, %s) = %s " % (motion, t,  mu[motion, t])
	if vis: print "\nmu%s = \n %s" % (it, mu)
	return mu


def E_step(it, E_mat, mu, data, params, sigma=None, vis=False ):
	"""
	returns: matrix of Expected values, shape (NxM) = (#of trajectories, #of motion behaviours)
		first is normalise so sum of expectances over the motion models equals 1
		second is pre-normalisation
	"""
	if vis: print "\n=====\nE STEP %s \n=====" % it
	num_of_motions = params["num_of_motions"]
	num_of_trajs = params["num_of_trajs"]
	num_of_timepoints = params["num_of_timepoints"]

	if sigma is None: sigma = 1
	(uuids, all_poses) = data

	for motion in xrange(0, num_of_motions):
		if vis: print "\nmotion: ", motion
		for cnt, (uuid, poses) in enumerate(zip(uuids, all_poses)):
			if cnt>=(num_of_trajs): continue

			#if cnt == 2: sys.exit(1)

			if vis: print "\nsubject: ", cnt
			center = mu[motion]
			product_these = 1

			for t in xrange(0, num_of_timepoints):
				mag = np.linalg.norm((center[t] - poses[t])) **2
				product_this = math.exp((-1/float(2 * sigma**2) * mag))

				if vis: print "time: %s, x = %s, mu_m[t] = %s, mag = %0.2f"\
					% (t, poses[t], center[t], mag)
				#print product_this
				#print product_these
				product_these *= product_this

			if vis: print "E_[c%s%s] = %s" % (cnt, motion, product_these)
			E_mat[cnt, motion] = product_these
		if vis: print E_mat[:, motion]

	#normalise E_mat so a row over all motions sum to 1
	norm_E_mat = E_mat.copy()
	for cnt, traj in enumerate(xrange(0, num_of_trajs)):
		s = sum(E_mat[traj, :])
		if s > 0: norm_E_mat[traj, :] = E_mat[traj, :] / s
		else: norm_E_mat[traj, :] = E_mat[traj, :]

	if vis: print "\nE_mat%s = \n %s" % (it, norm_E_mat)
	return norm_E_mat, E_mat


def monitoring_convergence(it, log_like, previous_ll, delta=0.1, vis=False):
	"""
	monitors the convergence of the EM algorithm:
		Previous Expecation matrix - New Expecation matrix  < delta = converged
	"""
	# todo: Check the data log likelihood score instead of the mean?
	print "\nTest Convergence... "

	if vis: print "%s, \n %s" % (previous_ll, log_like)
	if vis: print "difference in iter %s means: %s" % (it, (previous_ll-log_like))

	if abs(previous_ll-log_like) < delta:
		print ">>Converged."
		return True
	else:
		print ">>Not Converged. loop again..."
		return False


def test1_low_data_likelihood(un_normed_E_mat, E_mat, mu, poses, params, delta = 0.01, vis=False):
	"""
	For each trajectory test whether is has low likelihood for all motion patterns.
	"""
	print "\nTesting Likelihood of Trajectories (using Expectataions - before normalised)"
	if vis: print un_normed_E_mat
	converged_num_of_motion = params["num_of_motions"]

	most_likelihoods = [max(Ec_im) for Ec_im in un_normed_E_mat]
	least_likely = np.argmin(most_likelihoods)

	if most_likelihoods[least_likely] < delta:
		params["num_of_motions"] = converged_num_of_motion + 1

		if vis: print ">> Low Likely trajectory found: %s. Poses: %s. " % (least_likely, poses[least_likely])

		"""Add trajectory as new mean"""
		print "  added new mean = %s, max likelihood: %s" % (poses[least_likely], max(un_normed_E_mat[least_likely]))
		#x_new_mean = [x for x,y in poses[least_likely]]

		new_mu = mu.tolist()
		new_mu.append(poses[least_likely])
		mu = np.array(new_mu)
		if vis: print "new mu: \n%s " % mu

		"""Add new Expectations for the new mean"""
		new_E_mat = np.insert(un_normed_E_mat, converged_num_of_motion, delta, axis=1)
		if vis:print "new_E_mat: \n", new_E_mat
		E_mat = new_E_mat.copy()

		altered = True
	else:
		altered = False

	return E_mat, mu, altered, params


def test2_low_motion_utility(it, converged_likelihood, E_mat, mu, data, params, sigma, vis=False):
	"""calculate the motion pattern utility, which is the change in data log-likelihood
		with and without the specific motion pattern included.
	"""
	print "\ncalculating motion utility..."

	print "Current log likelihood with m=%s is %s " % (len(E_mat[0]), converged_likelihood)
	if vis: print params

	converged_num_of_motion = params["num_of_motions"]
	params["num_of_motions"] = converged_num_of_motion - 1

	if vis: print E_mat
	if vis: print mu
	all_likelihoods = [0]*(converged_num_of_motion+1)
	all_likelihoods[converged_num_of_motion] = converged_likelihood

	# When Testing the Motion Utilites: keep all E_mat and mu matrices.
	# The last possible will be the currently converged E_mat/mu.
	possible_E_mats = [E_mat]*(converged_num_of_motion+1)
	possible_mus = [mu]*(converged_num_of_motion+1)


	for m in xrange(0, converged_num_of_motion):
		print "\nremove motion: %s and recalculate..." %m
		cols = range(0, m)+range(m+1, converged_num_of_motion)
		#print "remaining m's = ", cols

		E_mat_reduced = E_mat[:,cols]
		mu_reduced = mu[cols,:]

		if vis: print E_mat_reduced
		if vis: print mu_reduced

		possible_mus[m] = mu_reduced
		possible_E_mats[m] = E_mat_reduced

		all_likelihoods[m] = data_log_likelihood(it, E_mat_reduced, mu_reduced, data, params, sigma, vis)

	best_model = np.argmax(all_likelihoods)
	if best_model == converged_num_of_motion: altered = False
	else: altered = True

	best_cols = range(0, best_model) + range(best_model +1, converged_num_of_motion)
	best_E_mat = E_mat[:,best_cols]
	best_mu = mu[best_cols,:]
	if vis:
		print best_cols
		print all_likelihoods
		print "BEST MODEL = %s" % best_model
		print converged_num_of_motion
		print best_E_mat
		print best_mu

	if not altered: print "\nDesicion: Converged Model is Best (with m = %s)" % best_model
	else: print "\nDesicion: remove motion", best_model
	return best_E_mat, best_mu, altered, params


def get_c_im(E_mat):
	"""
	Create c_im matrix which is binary for the classified motion pattern
	"""
	rows, cols = E_mat.shape
	c_im = np.zeros(rows*cols).reshape(rows, cols)

	for cnt, row in enumerate(E_mat):
		c_im[cnt, np.argmax(row)] = 1
	return c_im


def data_log_likelihood(it, E_mat, mu, data, params, sigma, vis=False):
	"""
	Returns the data log likelihood of an Expectation Matrix E_mat.
	Equation (6) in Bennewitz ICRA02
	"""
	if vis: print "\nIter: %s. Calculating data log likelihood..." % it

	num_of_motions = params["num_of_motions"]
	num_of_trajs = params["num_of_trajs"]
	num_of_timepoints = params["num_of_timepoints"]

	log = math.log(1/float(math.sqrt(2*math.pi)*sigma))
	constant = (num_of_timepoints*num_of_motions)*log
	one_over_2sigma_sq = 1/float(2*(sigma**2))

	(uuids, all_poses) = data

	c_im = get_c_im(E_mat)
	if vis: print "c_im = \n", c_im

	current_sum = 0
	for cnt, (uuid, poses) in enumerate(zip(uuids, all_poses)):
		if cnt>=(num_of_trajs): continue
		if vis: print "\nsubject: ", cnt

		traj_sum = 0

		motion = np.argmax(c_im[cnt])
		if vis: print "\n  motion: ", motion
		center = mu[motion]

		for t in xrange(0, num_of_timepoints):
			if vis: print "\n    timepoint: ", t

			mag = (np.linalg.norm(poses[t] - center[t]))**2

			if vis: print "    x[%s] = %s, mu[%s] = %s, mag = %s" % \
			              (t, poses[t], t, center[t], mag)

			traj_sum += c_im[cnt, motion] * mag
			if vis: print "    sum = ", traj_sum

		# print (one_over_2sigma_sq * traj_sum)
		traj_log_likelihood = (constant - (one_over_2sigma_sq * traj_sum))
		if vis: print "subj likelihood: ", traj_log_likelihood

		current_sum += traj_log_likelihood

	print "Iter = %s. Data log likelihood = %s " % (it, current_sum)
	return current_sum


def confusion_matrix_image(E_mat):
	#todo: Print the E_matrix as a confusion image, like in ICRA paper.
	return


def example_remove_motion():

	#Create some fake pose data
	fake_data = {1: [(1,1), (10,10)],
				 2: [(1,1), (9,9)],
				 3: [(4,4), (25,25)],
	             4: [(3,3), (20,20)]}

	params = {"num_of_motions": 3,
			  "num_of_trajs": 4,
			  "num_of_timepoints": 2}

	E_mat = np.array([[0.2, 0.6, 0.2],
			  		 [0.1, 0.4, 0.5],
					 [0.4, 0.3, 0.3],
	                 [0.8, 0.1, 0.1]])

	return fake_data, E_mat, params

def example_adding_motion():

	#Create some fake pose data
	fake_data = {1: [(1,1), (10,10)],
				 2: [(1,1), (9,9)],
				 3: [(4,4), (25,25)],
	             4: [(10,10), (1,1)]}

	params = {"num_of_motions": 2,
			  "num_of_trajs": 4,
			  "num_of_timepoints": 2}

	E_mat = np.array([[0.2, 0.8],
					  [0.6, 0.4],
					  [0.4, 0.6],
	                  [0.8, 0.2]])

	return fake_data, E_mat, params

def example_mini():

	#Create some fake pose data
	fake_data = {1: [(1,1), (10,10)],
				 2: [(1,1), (5,5)]}
	params = {"num_of_motions": 2,
			  "num_of_trajs": 2,
			  "num_of_timepoints": 2}

	E_mat = np.array([[0.6, 0.4],
					  [0.1, 0.8]])

	return fake_data, E_mat, params

def get_fake_data(which_data):
	if which_data is 1:
		fake_data, E_mat, params = example_mini()
	elif which_data is 2:
		fake_data, E_mat, params = example_remove_motion()
	elif which_data is 3:
		fake_data, E_mat, params = example_adding_motion()
	return fake_data, E_mat, params

def g4s_data(m_start, em_its, max_traj=None):

	data, max_number_of_poses = load_data(max_traj)

	#Fix the len of all poses:
	#file = '/home/strands/STRANDS/TESTING/metric_uuid_poses_append.p'
	extended_data = fix_data_length(data, max_number_of_poses)

	#Discretise data
	print "discretising data..."
	#todo: set the bin_size the same as our Qualitative Occu Grid
	discretised_data = discretise_data(data=extended_data, bin_size=1)

	uuids = discretised_data.keys()
	poses = [discretised_data[i] for i in uuids]
	data = (uuids, poses)

	#EM iterations
	num_of_motions = m_start                                        #This is M
	num_of_trajs = len(discretised_data.keys())                     #This is N
	num_of_timepoints = max_number_of_poses                         #This is T

	num_of_em_iterations = em_its

	params = {"num_of_motions": num_of_motions,
			  "num_of_trajs": num_of_trajs,
			  "num_of_timepoints": num_of_timepoints,
			  "num_of_em_iterations": num_of_em_iterations}
	print "params = ", params

	E_mat = E_init_step(params, vis=False)

	return data, E_mat, params


def EM_algorithm(which_data = None, m_start=10, em_its=100, max_traj=None):
	"""
	Run the EM algorithm until convergence
	"""

	if which_data is not None:
		data, E_mat, params = get_fake_data(which_data)
	else:
		data, E_mat, params = g4s_data(m_start, em_its, max_traj)
	uuids, poses = data

	"""Initialise Everything"""
	it = 0
	t1, t2, t3 = 0, 0, 0
	number_added, number_removed = 0, 0

	#print "\nE_mat^%s = \n%s" % (it, E_mat)
	mu_init = np.array([[(0.0,0.0)]*params["num_of_timepoints"]]*params["num_of_motions"])
	mu = M_step(it, E_mat, mu_init, data, params, vis=False)

	print "\nIteration: %s. m = %s. total = %s" % (it, params["num_of_motions"], mu.shape)
	likelihood = data_log_likelihood(it, E_mat, mu, data, params, sigma=1, vis=False)

	altered = True
	for it in xrange(1, params["num_of_em_iterations"]+1):

		if not altered: continue
		converged_in = it
		print "\nIteration: %s. m = %s." % (it, params["num_of_motions"])

		"""E_step"""
		st = time.time()
		E_mat, un_normed_E_mat = E_step(it, E_mat, mu, data, params, vis=False)
		t1 += time.time()-st

		"""M_step"""
		st = time.time()
		mu = M_step(it, E_mat, mu, data, params, vis=False)
		t2 += time.time()-st

		"""Data Likelihood"""
		st = time.time()
		previous_log_like = likelihood
		likelihood = data_log_likelihood(it, E_mat, mu, data, params, sigma=1, vis=False)
		t3 += time.time()-st

		"""Test Convergence"""
		if monitoring_convergence(it, likelihood, previous_log_like, delta=0.1, vis=False) and it>1:

			"""Test Trajectory Likelihoods"""
			e, m, altered, params = test1_low_data_likelihood(un_normed_E_mat, E_mat, mu, poses, params, vis=False)
			print ">>MOTION ADDED?", altered
			if altered: number_added+=1

			if not altered:
				"""Test Motion Utility"""
				e, m, altered, params = test2_low_motion_utility(it, likelihood, E_mat, mu, data, params, sigma=1, vis=False)

				"""Break from Loop"""
				print ">>MOTION REMOVED? ", altered
				if altered: number_removed+=1
			else:
				print "\nCONTINUE, but WITH NEW NUMBER OF MOTIONS:", params["num_of_motions"]

			E_mat = e.copy()
			mu = m.copy()

		print "Timings: E: %0.1f, M: %0.1f, ll: %0.1f" % (t1, t2, t3)

	print "\nConverged, and no better model found."
	print "\nRESULTS:   "
	print "E_mat = \n%s" % E_mat
	print "mu = \n%s" % mu
	print "c_im = \n%s" % get_c_im(E_mat)

	print "\nconverged fully in %s iterations" % converged_in
	print "motion updates: added: %s, removed %s." % (number_added, number_removed)



if __name__ == "__main__":
	which_data = None
	m_start = 2
	max_traj = 5
	max_em_its = 1000
	EM_algorithm(which_data, m_start, max_em_its, max_traj)
