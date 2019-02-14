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

#ifndef POINT_CLOUD_VISUALIZER_H
#define POINT_CLOUD_VISUALIZER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

class PointCloudVisualizer {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  PointCloudVisualizer();
  ~PointCloudVisualizer();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n);

  // Update the point cloud by appending an incremental point cloud.
  bool InsertPointCloud(const PointCloud& points);

  // Publish incrementally accumulated point clouds.
  void PublishIncrementalPointCloud();

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // The node's name.
  std::string name_;

  // World coordinate frame for publishing the point cloud.
  std::string fixed_frame_id_;

  // Most recent point cloud update time stamp for publishers.
  ros::Time stamp_;

  // Publishers.
  ros::Publisher incremental_points_pub_;

  // Store up incremental point clouds to be published when
  // PublishIncrementalPointCloud() is called. This makes it so that as the map
  // grows, we are only publishing new changes each time, making visualization
  // constant time rather than O(n).
  PointCloud::Ptr incremental_points_;

  // Enable or disable visualization. If this parameter is loaded as false, this
  // class object won't do anything.
  bool enable_visualization_;
};

#endif
