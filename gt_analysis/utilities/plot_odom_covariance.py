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


def extract_covariance(odom_msg):
  return np.array(odom_msg.pose.covariance).reshape(6, 6)


def plot_odom_covariance_maxmin(bag_file, odom_topic):
  rot_max_variances = []
  rot_min_variances = []
  tran_max_variances = []
  tran_min_variances = []
  with rosbag.Bag(bag_file) as bag:
    for topic, message, t in bag.read_messages(topics=[odom_topic]):
      cov = extract_covariance(message)
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


def plot_odom_covariance_xyz(bag_file, odom_topic):
  rot_variances_x = []
  rot_variances_y = []
  rot_variances_z = []
  tran_variances_x = []
  tran_variances_y = []
  tran_variances_z = []
  with rosbag.Bag(bag_file) as bag:
    for topic, message, t in bag.read_messages(topics=[odom_topic]):
      cov = extract_covariance(message)
      rot_variances_x.append(cov[0, 0])
      rot_variances_y.append(cov[1, 1])
      rot_variances_z.append(cov[2, 2])
      tran_variances_x.append(cov[3, 3])
      tran_variances_y.append(cov[4, 4])
      tran_variances_z.append(cov[5, 5])

  # plot
  plt.subplot(1, 2, 1)
  plt.plot(rot_variances_x, label="Variance x")
  plt.plot(rot_variances_y, label="Variance y")
  plt.plot(rot_variances_z, label="Variance z")
  plt.title("Rotation Variances")
  plt.legend()

  plt.subplot(1, 2, 2)
  plt.plot(tran_variances_x, label="Variance x")
  plt.plot(tran_variances_y, label="Variance y")
  plt.plot(tran_variances_z, label="Variance z")
  plt.title("Translation Variances")
  plt.legend()
  plt.show()

  return


def main():
    # Parse user input
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--topic_name", default="/husky1/lo_frontend/odometry", help="topic name")
    parser.add_argument("-b", "--bag", default="/home/costar/rosbag/beta2/husky4/lamp_01_fix_point_to_plane/odometry.bag", help="bag file")

    options = parser.parse_args()

    print("Reading %s from %s" % (options.topic_name, options.bag))

    plot_odom_covariance_maxmin(options.bag, options.topic_name)
    plot_odom_covariance_xyz(options.bag, options.topic_name)


if __name__ == '__main__':
    main()
