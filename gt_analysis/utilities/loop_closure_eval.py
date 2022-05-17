import yaml
import os
import copy
import pandas as pd
import numpy as np
import rospy 
import rosbag 
import sys
import csv 


import matplotlib.pyplot as plt

from evo.core import transformations
from evo.core import lie_algebra as lie

def quaternion_to_matrix(quaternion):
	# quaternion: np.array([qx, qy, qz, qw])
	x = quaternion[0]
	y = quaternion[1]
	z = quaternion[2]
	w = quaternion[3]
	Nq = w*w + x*x + y*y + z*z
	if Nq < np.finfo(np.float).eps:
		return np.eye(3)
	s = 2.0/Nq
	X = x*s
	Y = y*s
	Z = z*s
	wX = w*X; wY = w*Y; wZ = w*Z
	xX = x*X; xY = x*Y; xZ = x*Z
	yY = y*Y; yZ = y*Z; zZ = z*Z
	return np.array(
		   [[ 1.0-(yY+zZ), xY-wZ, xZ+wY ],
			[ xY+wZ, 1.0-(xX+zZ), yZ-wX ],
			[ xZ-wY, yZ+wX, 1.0-(xX+yY) ]])

def calculate_eigen_value(covariance):
    if len(covariance) == 36:
        cov_mat = np.array(covariance).reshape(6, 6)
        # if b_gtsam_cov:
        cov_mat = cov_mat[-3:,-3:]
    else:
        cov_mat = np.array(covariance).reshape(3, 3)
    s, v = np.linalg.eig(cov_mat)
    v[:, 0] /= np.linalg.norm(v[:, 0])
    v[:, 1] /= np.linalg.norm(v[:, 1])
    v[:, 2] /= np.linalg.norm(v[:, 2])

    if np.cross(v[:, 0], v[:, 1]).dot(v[:, 2]) < 0:
        # Make it right-handed
        v = v[:, [1, 0, 2]]
        s[0], s[1] = s[1], s[0]

    # Set ellipse scale
    k = 2.296  # 68.26%
    # k = 11.82  # 99.74%
    x_rad = k * np.sqrt(s[0])
    y_rad = k * np.sqrt(s[1])
    z_rad = k * np.sqrt(s[2])

    return x_rad, y_rad, z_rad

def key_to_chr_indx(key):
	key = np.uint64(key)
	keyBits = 64 * 8
	chrBits = 1 * 8
	indexBits = keyBits - chrBits
	chrMask = np.left_shift(np.uint64(255), np.uint64(indexBits))
	indexMask = ~chrMask

	char = np.right_shift((key & chrMask), np.uint64(indexBits))
	indx = key & indexMask

	return chr(char), indx

class LoopClosureEvaluator:
	def __init__(
		self, pose_graph_bag_name, pose_graph_topic, 
		loop_closure_bag_name, loop_closure_topic, 
		gt_odom_bag_name=None, gt_odom_topic=None):
		# pose_graph_bag: final pose graph including odom edges and inlier loop closure edges
		# loop_closure_bag: all detected loop closures
		# gt_odom_bag: ground truth odometry bag 

		# First parse bag into data 
		# pose graph 
		last_pose_graph_msg = None
		pose_graph_bag = rosbag.Bag(pose_graph_bag_name)
		for topic, msg, t in pose_graph_bag.read_messages(topics=[pose_graph_topic]):
			last_pose_graph_msg = msg

		self.nodes = dict() # {key: stamp}
		for node in last_pose_graph_msg.nodes:
			if node.ID == "odom_node":
				self.nodes[node.key] = node.header.stamp.to_nsec()
		
		self.inlier_loop_closures = []
		self.inlier_multirobot_lc = []
		self.inlier_singlerobot_lc = []
		self.inlier_pair_lc_map = {}
		for edge in last_pose_graph_msg.edges:
			if edge.type == 3:
				self.inlier_loop_closures.append((edge.key_from, edge.key_to))
				chr_from, idx_from = key_to_chr_indx(edge.key_from)
				chr_to, idx_to = key_to_chr_indx(edge.key_to)
				if (chr_from != chr_to):
					self.inlier_multirobot_lc.append((edge.key_from, edge.key_to))
				else:
					self.inlier_singlerobot_lc.append((edge.key_from, edge.key_to))
				if (chr_from, chr_to) in self.inlier_pair_lc_map:
					self.inlier_pair_lc_map[(chr_from, chr_to)].append(edge)
				elif (chr_to, chr_from) in self.inlier_pair_lc_map:
					self.inlier_pair_lc_map[(chr_to, chr_from)].append(edge)
				else:
					self.inlier_pair_lc_map[(chr_from, chr_to)] = [edge]
		
		# loop closure {(key_from, key_to): relative_transform}
		self.loop_closures = dict()
		self.multirobot_loop_closures = dict() 
		self.singlerobot_loop_closures = dict()
		self.pair_lc_map = {}

		# loop closure covariance {(key_from, key_to): covariance}
		self.loop_closure_covariances = dict() 
		loop_closure_bag = rosbag.Bag(loop_closure_bag_name)
		for topic, msg, t in loop_closure_bag.read_messages(topics=[loop_closure_topic]):
			for edge in msg.edges:
				translation = np.array([edge.pose.position.x, edge.pose.position.y, edge.pose.position.z])
				quaternion = np.array([edge.pose.orientation.x, edge.pose.orientation.y, 
					edge.pose.orientation.z, edge.pose.orientation.w])
				rotation = quaternion_to_matrix(quaternion)
				self.loop_closures[(edge.key_from, edge.key_to)] = lie.se3(rotation, translation)
				self.loop_closure_covariances[(edge.key_from, edge.key_to)] = edge.covariance
				chr_from, idx_from = key_to_chr_indx(edge.key_from)
				chr_to, idx_to = key_to_chr_indx(edge.key_to)
				if (chr_from != chr_to):
					self.multirobot_loop_closures[(edge.key_from, edge.key_to)] = lie.se3(rotation, translation)
				else:
					self.singlerobot_loop_closures[(edge.key_from, edge.key_to)] = lie.se3(rotation, translation)
				if (chr_from, chr_to) in self.pair_lc_map:
					self.pair_lc_map[(chr_from, chr_to)].append(edge)
				elif (chr_to, chr_from) in self.pair_lc_map:
					self.pair_lc_map[(chr_to, chr_from)].append(edge)
				else:
					self.pair_lc_map[(chr_from, chr_to)] = [edge]

		print("Total of {} loop closures detected, with {} inliers. ".format(len(self.loop_closures), len(self.inlier_loop_closures)))
		print("Total of {} inter-robot loop closures detected, with {} inliers. ".format(len(self.multirobot_loop_closures), len(self.inlier_multirobot_lc)))
		print("Total of {} single-robot loop closures detected, with {} inliers. ".format(len(self.singlerobot_loop_closures), len(self.inlier_singlerobot_lc)))

		print("LOOP CLOSURES BY ROBOT: \n")
		for pr in self.pair_lc_map:
			print("Between {r1} and {r2}, {nlc} loop closures".format(r1=str(pr[0]), r2=str(pr[1]), nlc=str(len(self.pair_lc_map[pr]))))

		print("INLIER LOOP CLOSURES BY ROBOT: \n")
		for pr in self.inlier_pair_lc_map:
			print("Between {r1} and {r2}, {nlc} loop closures".format(r1=str(pr[0]), r2=str(pr[1]), nlc=str(len(self.inlier_pair_lc_map[pr]))))
		
		self.lc_covar_radius = []
		for loop_closure in self.loop_closure_covariances:
			covar = self.loop_closure_covariances[loop_closure]
			x_rad, y_rad, z_rad = calculate_eigen_value(covar)
			cov_radius = np.sqrt(x_rad**2 + y_rad**2 + z_rad**2)
			self.lc_covar_radius.append(cov_radius)

		if gt_odom_bag_name != None and gt_odom_topic != None:
			self.gt_data = True
			# gt_odom
			self.gt_odom_stamps = []
			self.gt_odom_poses = []
			gt_odom_bag = rosbag.Bag(gt_odom_bag_name)
			for topic, msg, t in gt_odom_bag.read_messages(topics=[gt_odom_topic]):
				translation = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
				quaternion = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
					msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
				rotation = quaternion_to_matrix(quaternion)
				self.gt_odom_poses.append(lie.se3(rotation, translation))
				self.gt_odom_stamps.append(msg.header.stamp.to_nsec())

			self.gt_odom_stamps = np.asarray(self.gt_odom_stamps)
			self.gt_odom_poses = np.asarray(self.gt_odom_poses)

			self.accuracy_trans = []
			self.accuracy_rot = []
			self.inlier_accuracy_trans = []
			self.inlier_accuracy_rot = []

			# average loop closure accuracy
			self.avg_accuracy_trans = []
			self.avg_accuracy_rot = []
			self.avg_inlier_accuracy_trans = []
			self.avg_inlier_accuracy_rot = []
		else:
			self.gt_data = False

	def get_gt_relative_pose(self, stamp_from, stamp_to):
		# get idx of gt_odom closest to stamps
		idx_from = (np.abs(self.gt_odom_stamps - stamp_from)).argmin()
		idx_to = (np.abs(self.gt_odom_stamps - stamp_to)).argmin()
		pose_from = self.gt_odom_poses[idx_from]
		pose_to = self.gt_odom_poses[idx_to]
		return lie.relative_se3(pose_from, pose_to)
	
	# TODO: add normalized evaluation!!! 
	def evaluate(self):
		if not self.gt_data:
			print("No ground truth data. Cannot evaluate accuracy. ")
			return
		if len(self.loop_closures) == 0:
			print("No loop closures detected. ")
			return
		# first evaluate accuracy of all loop closures 
		for loop_closure in self.loop_closures:
			inlier = loop_closure in self.inlier_loop_closures
			stamp_from = self.nodes[loop_closure[0]]
			stamp_to = self.nodes[loop_closure[1]]
			gt_transform = self.get_gt_relative_pose(stamp_from, stamp_to)
			est_transform = self.loop_closures[loop_closure]

			# get difference between gt and est transform 
			error = lie.relative_se3(gt_transform, est_transform)
			error_trans = np.linalg.norm(error[:3, 3])
			error_rot = 180 / np.pi * abs(lie.so3_log(error[:3, :3]))

			self.accuracy_trans.append(error_trans)
			self.accuracy_rot.append(error_rot)
			self.avg_accuracy_trans.append(error_trans / abs(loop_closure[1] - loop_closure[0]))
			self.avg_accuracy_rot.append(error_rot / abs(loop_closure[1] - loop_closure[0]))
			if (inlier):
				self.inlier_accuracy_trans.append(error_trans)
				self.inlier_accuracy_rot.append(error_rot)
				self.avg_inlier_accuracy_trans.append(error_trans / abs(loop_closure[1] - loop_closure[0]))
				self.avg_inlier_accuracy_rot.append(error_rot / abs(loop_closure[1] - loop_closure[0]))

		return

	def get_num_loopclosures(self):
		return len(self.loop_closures)

	def get_num_inliers(self):
		return len(self.inlier_loop_closures)

	def get_accuracy_trans(self):
		# get loop closure accuracy for all loop closures 
		accuracy = np.asarray(self.accuracy_trans)
		print("Loop closure translation accuracy. median: {}, mean: {}, "
			"min: {}, max: {}".format(np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)))
		return np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)

	def get_accuracy_rot(self):
		# get loop closure accuracy for all loop closures 
		accuracy = np.asarray(self.accuracy_rot)
		print("Loop closure rotation accuracy. median: {}, mean: {}, "
			"min: {}, max: {}".format(np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)))
		return np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)

	def get_inlier_accuracy_trans(self):
		# get inlier loop closure accuracy for all loop closures 
		accuracy = np.asarray(self.inlier_accuracy_trans)
		print("Inlier loop closure translation accuracy. median: {}, mean: {}, "
			"min: {}, max: {}".format(np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)))
		return np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)

	def get_inlier_accuracy_rot(self):
		# get loop closure accuracy for all loop closures 
		accuracy = np.asarray(self.inlier_accuracy_rot)
		print("Inlier loop closure rotation accuracy. median: {}, mean: {}, "
			"min: {}, max: {}".format(np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)))
		return np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)

	def get_avg_accuracy_trans(self):
		# get loop closure accuracy for all loop closures 
		accuracy = np.asarray(self.avg_accuracy_trans)
		print("Avg (per node) loop closure translation accuracy. median: {}, mean: {}, "
			"min: {}, max: {}".format(np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)))
		return np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)

	def get_avg_accuracy_rot(self):
		# get loop closure accuracy for all loop closures 
		accuracy = np.asarray(self.avg_accuracy_rot)
		print("Avg (per node) loop closure rotation accuracy. median: {}, mean: {}, "
			"min: {}, max: {}".format(np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)))
		return np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)

	def get_avg_inlier_accuracy_trans(self):
		# get inlier loop closure accuracy for all loop closures 
		accuracy = np.asarray(self.avg_inlier_accuracy_trans)
		print("Avg (per node) inlier loop closure translation accuracy. median: {}, mean: {}, "
			"min: {}, max: {}".format(np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)))
		return np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)

	def get_avg_inlier_accuracy_rot(self):
		# get loop closure accuracy for all loop closures 
		accuracy = np.asarray(self.avg_inlier_accuracy_rot)
		print("Avg (per node) inlier oop closure rotation accuracy. median: {}, mean: {}, "
			"min: {}, max: {}".format(np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)))
		return np.median(accuracy), np.mean(accuracy), np.min(accuracy), np.max(accuracy)

	def save_accuracy_plot(self, filename="", save=True):
		# save plot for loop closure accuracy plot
		fig, (ax1, ax2) = plt.subplots(2)
		fig.suptitle('Loop closure error')
		median, mean, _, _ = self.get_accuracy_trans()
		ax1.plot(self.accuracy_trans)
		ax1.hlines(mean, 0, len(self.accuracy_trans) - 1, color='r', label="mean")
		ax1.hlines(median, 0, len(self.accuracy_trans) - 1, color='b', label="median")
		ax1.legend()
		ax1.set_title("Loop closure translation error (m)", fontsize="medium")
		ax1.xaxis.set_tick_params(labelsize='medium')
		ax1.yaxis.set_tick_params(labelsize='medium')
		median, mean, _, _ = self.get_accuracy_rot()
		ax2.plot(self.accuracy_rot)
		ax2.hlines(mean, 0, len(self.accuracy_rot) - 1, color='r', label="mean")
		ax2.hlines(median, 0, len(self.accuracy_rot) - 1, color='b', label="median")
		ax2.legend()
		ax2.set_title("Loop closure rotation error (deg)", fontsize="medium")
		ax2.xaxis.set_tick_params(labelsize='medium')
		ax2.yaxis.set_tick_params(labelsize='medium')

		if not save:
			plt.show()
		else:
			plt.savefig(filename, bbox_inches='tight')
		return

	def save_avg_accuracy_plot(self, filename="", save=True):
		# save plot for loop closure accuracy plot
		fig, (ax1, ax2) = plt.subplots(2)
		fig.suptitle('Avg (per node) loop closure error')
		median, mean, _, _ = self.get_avg_accuracy_trans()
		ax1.plot(self.avg_accuracy_trans)
		ax1.hlines(mean, 0, len(self.avg_accuracy_trans) - 1, color='r', label="mean")
		ax1.hlines(median, 0, len(self.avg_accuracy_trans) - 1, color='b', label="median")
		ax1.legend()
		ax1.set_title("Avg (per node) loop closure translation error (m)", fontsize="medium")
		ax1.xaxis.set_tick_params(labelsize='medium')
		ax1.yaxis.set_tick_params(labelsize='medium')
		median, mean, _, _ = self.get_avg_accuracy_rot()
		ax2.plot(self.avg_accuracy_rot)
		ax2.hlines(mean, 0, len(self.avg_accuracy_rot) - 1, color='r', label="mean")
		ax2.hlines(median, 0, len(self.avg_accuracy_rot) - 1, color='b', label="median")
		ax2.legend()
		ax2.set_title("Avg (per node) loop closure rotation error (deg)", fontsize="medium")
		ax2.xaxis.set_tick_params(labelsize='medium')
		ax2.yaxis.set_tick_params(labelsize='medium')

		if not save:
			plt.show()
		else:
			plt.savefig(filename, bbox_inches='tight')
		return

	def save_inlier_accuracy_plot(self, filename="", save=True):
		# save plot for inlier loop closure accuracy
		fig, (ax1, ax2) = plt.subplots(2)
		fig.suptitle('Loop closure inliers error')
		median, mean, _, _ = self.get_inlier_accuracy_trans()
		ax1.plot(self.inlier_accuracy_trans)
		ax1.hlines(mean, 0, len(self.inlier_accuracy_trans) - 1, color='r', label="mean")
		ax1.hlines(median, 0, len(self.inlier_accuracy_trans) - 1, color='b', label="median")
		ax1.legend()
		ax1.set_title("Loop closure inliers translation error (m)", fontsize="medium")
		ax1.xaxis.set_tick_params(labelsize='medium')
		ax1.yaxis.set_tick_params(labelsize='medium')
		median, mean, _, _ = self.get_inlier_accuracy_rot()
		ax2.plot(self.inlier_accuracy_rot)
		ax2.hlines(mean, 0, len(self.inlier_accuracy_rot) - 1, color='r', label="mean")
		ax2.hlines(median, 0, len(self.inlier_accuracy_rot) - 1, color='b', label="median")
		ax2.legend()
		ax2.set_title("Loop closure inliers rotation error (deg)", fontsize="medium")
		ax2.xaxis.set_tick_params(labelsize='medium')
		ax2.yaxis.set_tick_params(labelsize='medium')

		if not save:
			plt.show()
		else:
			plt.savefig(filename, bbox_inches='tight')
		return

	def save_avg_inlier_accuracy_plot(self, filename="", save=True):
		# save plot for inlier loop closure accuracy
		fig, (ax1, ax2) = plt.subplots(2)
		fig.suptitle('Avg (per node) loop closure inliers error')
		median, mean, _, _ = self.get_avg_inlier_accuracy_trans()
		ax1.plot(self.avg_inlier_accuracy_trans)
		ax1.hlines(mean, 0, len(self.avg_inlier_accuracy_trans) - 1, color='r', label="mean")
		ax1.hlines(median, 0, len(self.avg_inlier_accuracy_trans) - 1, color='b', label="median")
		ax1.legend()
		ax1.set_title("Avg (per node) loop closure inliers translation error (m)", fontsize="medium")
		ax1.xaxis.set_tick_params(labelsize='medium')
		ax1.yaxis.set_tick_params(labelsize='medium')
		median, mean, _, _ = self.get_avg_inlier_accuracy_rot()
		ax2.plot(self.avg_inlier_accuracy_rot)
		ax2.hlines(mean, 0, len(self.avg_inlier_accuracy_rot) - 1, color='r', label="mean")
		ax2.hlines(median, 0, len(self.avg_inlier_accuracy_rot) - 1, color='b', label="median")
		ax2.legend()
		ax2.set_title("Avg (per node) loop closure inliers rotation error (deg)", fontsize="medium")
		ax2.xaxis.set_tick_params(labelsize='medium')
		ax2.yaxis.set_tick_params(labelsize='medium')

		if not save:
			plt.show()
		else:
			plt.savefig(filename, bbox_inches='tight')
		return

	def save_stats(self, results_dir):
		with open(results_dir + "lc_stats.csv", 'w') as outfile:
			writer = csv.writer(outfile, delimiter=",", quotechar='|', quoting=csv.QUOTE_MINIMAL)
			writer.writerow(['# total loop closures', '# total inliers', ])
			writer.writerow([len(self.loop_closures), len(self.inlier_loop_closures)])

	def save_covariance_plot(self, filename="", save=True):
		fig, ax = plt.subplots()
		ax.plot(self.lc_covar_radius)
		fig.suptitle("Loop Closure Covariance Radius")
		ax.xaxis.set_tick_params(labelsize='medium')
		ax.yaxis.set_tick_params(labelsize='medium')
		if not save:
			plt.show()
		else:
			plt.savefig(filename, bbox_inches='tight')
		return

if __name__=="__main__":
	if len(sys.argv) < 5:
		print("Example Usage: python loop_closure_eval.py pose_graph.bag pose_graph_topic " 
			"loop_closure.bag loop_closure_topic gt_odometry.bag gt_odometry_topic")
		sys.exit(1)
	eval_accuracy = False
	if len(sys.argv) < 7:
		eval = LoopClosureEvaluator(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
	else:
		eval = LoopClosureEvaluator(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6])
		eval_accuracy = True
	eval.evaluate()
	if eval_accuracy:
		eval.save_accuracy_plot(save=False)
		eval.save_inlier_accuracy_plot(save=False)
		eval.save_avg_accuracy_plot(save=False)
		eval.save_avg_inlier_accuracy_plot(save=False)
