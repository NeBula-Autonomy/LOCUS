#!/usr/bin/python
### plot covariance of odometry edges
import pdb
import matplotlib.pyplot as plt
import matplotlib
from nav_msgs.msg import Odometry
import rosbag
import rospy
import argparse
import enum
import math
import os
import numpy as np


def get_max_diagonal_term(matrix):
  return max(matrix.diagonal())


def get_min_diagonal_term(matrix):
  return min(matrix.diagonal())


def calculate_eigen_values(covariance):
    s, v = np.linalg.eig(covariance)
    return s, v


def extract_covariance(pose_graph_edge):
  return np.array(pose_graph_edge.covariance).reshape(6, 6)


def plot_factor_covariance_maxmin(bag_file, topic, edge_type):
  rot_max_variances = []
  rot_min_variances = []
  tran_max_variances = []
  tran_min_variances = []
  last_message = None
  # Extract last pose graph message
  with rosbag.Bag(bag_file) as bag:
    for topic, message, t in bag.read_messages(topics=[topic]):
      last_message = message

  for edge in last_message.edges:
    if edge.type == edge_type:
      cov = extract_covariance(edge)
      rot_max_var = get_max_diagonal_term(cov[:3, :3])
      rot_min_var = get_min_diagonal_term(cov[:3, :3])
      rot_max_variances.append(rot_max_var)
      rot_min_variances.append(rot_min_var)
      tran_max_var = get_max_diagonal_term(cov[-3:, -3:])
      tran_min_var = get_min_diagonal_term(cov[-3:, -3:])
      tran_max_variances.append(tran_max_var)
      tran_min_variances.append(tran_min_var)

  # plot
  plt.subplot(1, 2, 1)
  plt.plot(rot_min_variances, label="Min Variance")
  plt.plot(rot_max_variances, label="Max Variance")
  plt.title("Rotation Variances")
  plt.legend()

  plt.subplot(1, 2, 2)
  plt.plot(tran_min_variances, label="Min Variance")
  plt.plot(tran_max_variances, label="Max Variance")
  plt.title("Translation Variances")
  plt.legend()
  plt.show()

  return


def plot_factor_covariance_xyz(bag_file, topic, edge_type):
  rot_variances_x = []
  rot_variances_y = []
  rot_variances_z = []
  tran_variances_x = []
  tran_variances_y = []
  tran_variances_z = []
  last_message = None
  with rosbag.Bag(bag_file) as bag:
    for topic, message, t in bag.read_messages(topics=[topic]):
      last_message = message

  for edge in last_message.edges:
    if edge.type == edge_type:
      cov = extract_covariance(edge)
      rot_variances_x.append(np.sqrt(cov[0, 0]))
      rot_variances_y.append(np.sqrt(cov[1, 1]))
      rot_variances_z.append(np.sqrt(cov[2, 2]))
      tran_variances_x.append(np.sqrt(cov[3, 3]))
      tran_variances_y.append(np.sqrt(cov[4, 4]))
      tran_variances_z.append(np.sqrt(cov[5, 5]))

  # plot
  plt.subplot(1, 2, 1)
  plt.plot(rot_variances_x, label="Std Dev x")
  plt.plot(rot_variances_y, label="Std Dev y")
  plt.plot(rot_variances_z, label="Std Dev z")
  plt.title("Rotation Std Dev")
  plt.legend()

  plt.subplot(1, 2, 2)
  plt.plot(tran_variances_x, label="Std Dev x")
  plt.plot(tran_variances_y, label="Std Dev y")
  plt.plot(tran_variances_z, label="Std Dev z")
  plt.title("Translation Std Dev")
  plt.legend()
  plt.show()

  return


def main():
    # Parse user input
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic_name", default="/husky1/lamp/pose_graph", help="topic name")
    parser.add_argument("-b", "--bag", default="/home/costar/rosbag/beta2/husky4/lamp_01_fix_point_to_plane/pose_graph.bag", help="bag file")
    parser.add_argument("-e", "--edge_type", type=int, default=0, help="edge type")

    options = parser.parse_args()
    print("Reading %s from %s and plotting covariance for edge type %s" % (options.topic_name, options.bag, str(options.edge_type)))

    plot_factor_covariance_maxmin(options.bag, options.topic_name, options.edge_type)
    plot_factor_covariance_xyz(options.bag, options.topic_name, options.edge_type)


if __name__ == '__main__':
    main()
