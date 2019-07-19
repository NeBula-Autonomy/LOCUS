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

#ifndef POINT_CLOUD_FILTER_H
#define POINT_CLOUD_FILTER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

class PointCloudFilter {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  PointCloudFilter();
  ~PointCloudFilter();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n);

  // Filter an incoming point cloud
  bool Filter(const PointCloud::ConstPtr& points,
              PointCloud::Ptr points_filtered) const;

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // The node's name.
  std::string name_;

  struct Parameters {
    // Apply a voxel grid filter.
    bool grid_filter;

    // Resolution of voxel grid filter.
    double grid_res;

    // Apply a random downsampling filter.
    bool random_filter;

    // Percentage of points to discard. Must be between 0.0 and 1.0;
    double decimate_percentage;

    // Apply a statistical outlier filter.
    bool outlier_filter;

    // Standard deviation threshold in distance to neighbors for outlier
    // removal.
    double outlier_std;

    // Number of nearest neighbors to use for outlier filter.
    unsigned int outlier_knn;

    // Apply a radius outlier filter.
    bool radius_filter;

    // Size of the radius filter.
    double radius;

    // If this number of neighbors are not found within a radius around each
    // point, remove that point.
    unsigned int radius_knn;
  } params_;
};

#endif
