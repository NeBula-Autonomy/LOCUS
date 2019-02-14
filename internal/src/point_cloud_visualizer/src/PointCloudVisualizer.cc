/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <point_cloud_visualizer/PointCloudVisualizer.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pu = parameter_utils;

PointCloudVisualizer::PointCloudVisualizer() {
  // Instantiate point cloud pointers.
  incremental_points_.reset(new PointCloud);
}

PointCloudVisualizer::~PointCloudVisualizer() {}

bool PointCloudVisualizer::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudVisualizer");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool PointCloudVisualizer::LoadParameters(const ros::NodeHandle& n) {
  // Load visualization parameters.
  if (!pu::Get("visualizer/enable_visualization", enable_visualization_))
    return false;

  // Load coordinate frames.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;

  return true;
}

bool PointCloudVisualizer::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Initialize publishers.
  incremental_points_pub_ =
      nl.advertise<sensor_msgs::PointCloud2>("incremental_points", 10, false);

  return true;
}

bool PointCloudVisualizer::InsertPointCloud(const PointCloud& points) {
  // Make sure the user wants visualization enabled.
  if (!enable_visualization_)
    return false;

  // Store the timestamp for the next time the point cloud is published.
  stamp_.fromNSec(points.header.stamp*1e3);

  // Merge with the existing point cloud .
  *incremental_points_ += points;

  return true;
}

void PointCloudVisualizer::PublishIncrementalPointCloud() {
  // Check before doing any work.
  if (!enable_visualization_ ||
      incremental_points_pub_.getNumSubscribers() == 0)
    return;

  // Convert incremental points to ROS's sensor_msgs::PointCloud2 type.
  sensor_msgs::PointCloud2 ros_incremental_points;
  pcl::toROSMsg(*incremental_points_, ros_incremental_points);
  ros_incremental_points.header.stamp = stamp_;
  ros_incremental_points.header.frame_id = fixed_frame_id_;
  incremental_points_pub_.publish(ros_incremental_points);

  // Remove all incremental points to reset accumulation.
  incremental_points_.reset(new PointCloud);
}
