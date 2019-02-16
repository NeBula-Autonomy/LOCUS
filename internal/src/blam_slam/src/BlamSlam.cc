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

#include <blam_slam/BlamSlam.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl_conversions/pcl_conversions.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

BlamSlam::BlamSlam()
    : estimate_update_rate_(0.0), visualization_update_rate_(0.0) {}

BlamSlam::~BlamSlam() {}

bool BlamSlam::Initialize(const ros::NodeHandle& n, bool from_log) {
  name_ = ros::names::append(n.getNamespace(), "BlamSlam");

  if (!filter_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }

  if (!odometry_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud odometry.", name_.c_str());
    return false;
  }

  if (!loop_closure_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize laser loop closure.", name_.c_str());
    return false;
  }

  if (!localization_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize localization.", name_.c_str());
    return false;
  }

  if (!mapper_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize mapper.", name_.c_str());
    return false;
  }

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n, from_log)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool BlamSlam::LoadParameters(const ros::NodeHandle& n) {
  // Load update rates.
  if (!pu::Get("rate/estimate", estimate_update_rate_)) return false;
  if (!pu::Get("rate/visualization", visualization_update_rate_)) return false;

  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;
  if (!pu::Get("frame_id/base", base_frame_id_)) return false;

  return true;
}

bool BlamSlam::RegisterCallbacks(const ros::NodeHandle& n, bool from_log) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  visualization_update_timer_ = nl.createTimer(
      visualization_update_rate_, &BlamSlam::VisualizationTimerCallback, this);

  if (from_log)
    return RegisterLogCallbacks(n);
  else
    return RegisterOnlineCallbacks(n);
}

bool BlamSlam::RegisterLogCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering log callbacks.", name_.c_str());
  return CreatePublishers(n);
}

bool BlamSlam::RegisterOnlineCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering online callbacks.", name_.c_str());

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  estimate_update_timer_ = nl.createTimer(
      estimate_update_rate_, &BlamSlam::EstimateTimerCallback, this);

  pcld_sub_ = nl.subscribe("pcld", 100, &BlamSlam::PointCloudCallback, this);

  return CreatePublishers(n);
}

bool BlamSlam::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  base_frame_pcld_pub_ =
      nl.advertise<PointCloud>("base_frame_point_cloud", 10, false);

  return true;
}

void BlamSlam::PointCloudCallback(const PointCloud::ConstPtr& msg) {
  synchronizer_.AddPCLPointCloudMessage(msg);
}

void BlamSlam::EstimateTimerCallback(const ros::TimerEvent& ev) {
  // Sort all messages accumulated since the last estimate update.
  synchronizer_.SortMessages();

  // Iterate through sensor messages, passing to update functions.
  MeasurementSynchronizer::sensor_type type;
  unsigned int index = 0;
  while (synchronizer_.GetNextMessage(&type, &index)) {
    switch(type) {

      // Point cloud messages.
      case MeasurementSynchronizer::PCL_POINTCLOUD: {
        const MeasurementSynchronizer::Message<PointCloud>::ConstPtr& m =
            synchronizer_.GetPCLPointCloudMessage(index);

        ProcessPointCloudMessage(m->msg);
        break;
      }

      // Unhandled sensor messages.
      default: {
        ROS_WARN("%s: Unhandled measurement type (%s).", name_.c_str(),
                 MeasurementSynchronizer::GetTypeString(type).c_str());
        break;
      }
    }
  }

  // Remove processed messages from the synchronizer.
  synchronizer_.ClearMessages();
}

void BlamSlam::VisualizationTimerCallback(const ros::TimerEvent& ev) {
  mapper_.PublishMap();
}

void BlamSlam::ProcessPointCloudMessage(const PointCloud::ConstPtr& msg) {
  // Filter the incoming point cloud message.
  PointCloud::Ptr msg_filtered(new PointCloud);
  filter_.Filter(msg, msg_filtered);

  // Update odometry by performing ICP.
  if (!odometry_.UpdateEstimate(*msg_filtered)) {
    // First update ever.
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(msg_filtered, unused.get());
    loop_closure_.AddKeyScanPair(0, msg);
    return;
  }

  // Containers.
  PointCloud::Ptr msg_transformed(new PointCloud);
  PointCloud::Ptr msg_neighbors(new PointCloud);
  PointCloud::Ptr msg_base(new PointCloud);
  PointCloud::Ptr msg_fixed(new PointCloud);

  // Transform the incoming point cloud to the best estimate of the base frame.
  localization_.MotionUpdate(odometry_.GetIncrementalEstimate());
  localization_.TransformPointsToFixedFrame(*msg_filtered,
                                            msg_transformed.get());

  // Get approximate nearest neighbors from the map.
  mapper_.ApproxNearestNeighbors(*msg_transformed, msg_neighbors.get());

  // Transform those nearest neighbors back into sensor frame to perform ICP.
  localization_.TransformPointsToSensorFrame(*msg_neighbors, msg_neighbors.get());

  // Localize to the map. Localization will output a pointcloud aligned in the
  // sensor frame.
  localization_.MeasurementUpdate(msg_filtered, msg_neighbors, msg_base.get());

  // Check for new loop closures.
  bool new_keyframe;
  if (HandleLoopClosures(msg, &new_keyframe)) {
    // We found one - regenerate the 3D map.
    PointCloud::Ptr regenerated_map(new PointCloud);
    loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

    mapper_.Reset();
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(regenerated_map, unused.get());

    // Also reset the robot's estimated position.
    localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());
  } else {
    // No new loop closures - but was there a new key frame? If so, add new
    // points to the map.
    if (new_keyframe) {
      localization_.MotionUpdate(gu::Transform3::Identity());
      localization_.TransformPointsToFixedFrame(*msg, msg_fixed.get());
      PointCloud::Ptr unused(new PointCloud);
      mapper_.InsertPoints(msg_fixed, unused.get());
    }
  }

  // Visualize the pose graph and current loop closure radius.
  loop_closure_.PublishPoseGraph();

  // Publish the incoming point cloud message from the base frame.
  if (base_frame_pcld_pub_.getNumSubscribers() != 0) {
    PointCloud base_frame_pcld = *msg;
    base_frame_pcld.header.frame_id = base_frame_id_;
    base_frame_pcld_pub_.publish(base_frame_pcld);
  }
}

bool BlamSlam::HandleLoopClosures(const PointCloud::ConstPtr& scan,
                                  bool* new_keyframe) {
  if (new_keyframe == NULL) {
    ROS_ERROR("%s: Output boolean for new keyframe is null.", name_.c_str());
    return false;
  }

  // Add the new pose to the pose graph.
  unsigned int pose_key;
  gu::MatrixNxNBase<double, 6> covariance;
  covariance.Zeros();
  for (int i = 0; i < 3; ++i)
    covariance(i, i) = 0.01;
  for (int i = 3; i < 6; ++i)
    covariance(i, i) = 0.004;

  const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
  if (!loop_closure_.AddBetweenFactor(localization_.GetIncrementalEstimate(),
                                      covariance, stamp, &pose_key)) {
    return false;
  }
  *new_keyframe = true;

  if (!loop_closure_.AddKeyScanPair(pose_key, scan)) {
    return false;
  }

  std::vector<unsigned int> closure_keys;
  if (!loop_closure_.FindLoopClosures(pose_key, &closure_keys)) {
    return false;
  }

  for (const auto& closure_key : closure_keys) {
    ROS_INFO("%s: Closed loop between poses %u and %u.", name_.c_str(),
             pose_key, closure_key);
  }
  return true;
}
