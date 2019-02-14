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

#include <point_cloud_odometry/PointCloudOdometry.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/registration/gicp.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

using pcl::copyPointCloud;
using pcl::GeneralizedIterativeClosestPoint;
using pcl::PointCloud;
using pcl::PointXYZ;

PointCloudOdometry::PointCloudOdometry() : initialized_(false) {
  query_.reset(new PointCloud);
  reference_.reset(new PointCloud);
}

PointCloudOdometry::~PointCloudOdometry() {}

bool PointCloudOdometry::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudOdometry");

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

bool PointCloudOdometry::LoadParameters(const ros::NodeHandle& n) {
  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;
  if (!pu::Get("frame_id/odometry", odometry_frame_id_)) return false;

  // Load initial position.
  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  if (!pu::Get("init/position/x", init_x)) return false;
  if (!pu::Get("init/position/y", init_y)) return false;
  if (!pu::Get("init/position/z", init_z)) return false;
  if (!pu::Get("init/orientation/roll", init_roll)) return false;
  if (!pu::Get("init/orientation/pitch", init_pitch)) return false;
  if (!pu::Get("init/orientation/yaw", init_yaw)) return false;

  gu::Transform3 init;
  init.translation = gu::Vec3(init_x, init_y, init_z);
  init.rotation = gu::Rot3(init_roll, init_pitch, init_yaw);
  integrated_estimate_ = init;

  // Load algorithm parameters.
  if (!pu::Get("icp/tf_epsilon", params_.icp_tf_epsilon)) return false;
  if (!pu::Get("icp/corr_dist", params_.icp_corr_dist)) return false;
  if (!pu::Get("icp/iterations", params_.icp_iterations)) return false;

  if (!pu::Get("icp/transform_thresholding", transform_thresholding_)) return false;
  if (!pu::Get("icp/max_translation", max_translation_)) return false;
  if (!pu::Get("icp/max_rotation", max_rotation_)) return false;

  return true;
}

bool PointCloudOdometry::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  query_pub_ = nl.advertise<PointCloud>("odometry_query_points", 10, false);
  reference_pub_ =
      nl.advertise<PointCloud>("odometry_reference_points", 10, false);
  incremental_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "odometry_incremental_estimate", 10, false);
  integrated_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "odometry_integrated_estimate", 10, false);

  return true;
}

bool PointCloudOdometry::UpdateEstimate(const PointCloud& points) {
  // Store input point cloud's time stamp for publishing.
  stamp_.fromNSec(points.header.stamp*1e3);

  // If this is the first point cloud, store it and wait for another.
  if (!initialized_) {
    copyPointCloud(points, *query_);
    initialized_ = true;
    return false;
  }

  // Move current query points (acquired last iteration) to reference points.
  copyPointCloud(*query_, *reference_);

  // Set the incoming point cloud as the query point cloud.
  copyPointCloud(points, *query_);

  // Update pose estimate via ICP.
  return UpdateICP();
}

const gu::Transform3& PointCloudOdometry::GetIncrementalEstimate() const {
  return incremental_estimate_;
}

const gu::Transform3& PointCloudOdometry::GetIntegratedEstimate() const {
  return integrated_estimate_;
}

bool PointCloudOdometry::GetLastPointCloud(PointCloud::Ptr& out) const {
  if (!initialized_ || query_ == NULL) {
    ROS_WARN("%s: Not initialized.", name_.c_str());
    return false;
  }

  out = query_;
  return true;
}

bool PointCloudOdometry::UpdateICP() {
  // Compute the incremental transformation.
  GeneralizedIterativeClosestPoint<PointXYZ, PointXYZ> icp;
  icp.setTransformationEpsilon(params_.icp_tf_epsilon);
  icp.setMaxCorrespondenceDistance(params_.icp_corr_dist);
  icp.setMaximumIterations(params_.icp_iterations);
  icp.setRANSACIterations(0);

  icp.setInputSource(query_);
  icp.setInputTarget(reference_);

  PointCloud unused_result;
  icp.align(unused_result);

  const Eigen::Matrix4f T = icp.getFinalTransformation();

  // Update pose estimates.
  incremental_estimate_.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  incremental_estimate_.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                            T(1, 0), T(1, 1), T(1, 2),
                                            T(2, 0), T(2, 1), T(2, 2));

  // Only update if the incremental transform is small enough.
  if (!transform_thresholding_ ||
      (incremental_estimate_.translation.Norm() <= max_translation_ &&
       incremental_estimate_.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    integrated_estimate_ =
        gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  } else {
    ROS_WARN(
        "%s: Discarding incremental transformation with norm (t: %lf, r: %lf)",
        name_.c_str(), incremental_estimate_.translation.Norm(),
        incremental_estimate_.rotation.ToEulerZYX().Norm());
  }

  // Convert pose estimates to ROS format and publish.
  PublishPose(incremental_estimate_, incremental_estimate_pub_);
  PublishPose(integrated_estimate_, integrated_estimate_pub_);

  // Publish point clouds for visualization.
  PublishPoints(query_, query_pub_);
  PublishPoints(reference_, reference_pub_);

  // Convert transform between fixed frame and odometry frame.
  geometry_msgs::TransformStamped tf;
  tf.transform = gr::ToRosTransform(integrated_estimate_);
  tf.header.stamp = stamp_;
  tf.header.frame_id = fixed_frame_id_;
  tf.child_frame_id = odometry_frame_id_;
  tfbr_.sendTransform(tf);

  return true;
}

void PointCloudOdometry::PublishPoints(const PointCloud::Ptr& points,
                                       const ros::Publisher& pub) {
  // Check for subscribers before doing any work.
  if (pub.getNumSubscribers() > 0) {
    PointCloud out;
    out = *points;
    out.header.frame_id = odometry_frame_id_;
    pub.publish(out);
  }
}

void PointCloudOdometry::PublishPose(const gu::Transform3& pose,
                                     const ros::Publisher& pub) {
  // Check for subscribers before doing any work.
  if (pub.getNumSubscribers() == 0)
    return;

  // Convert from gu::Transform3 to ROS's PoseStamped type and publish.
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.pose = gr::ToRosPose(pose);
  ros_pose.header.frame_id = fixed_frame_id_;
  ros_pose.header.stamp = stamp_;
  pub.publish(ros_pose);
}
