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

#include <lo_frontend/LoFrontend.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

LoFrontend::LoFrontend()
  : estimate_update_rate_(0.0), visualization_update_rate_(0.0),
  b_add_first_scan_to_key_(true), translation_threshold_kf_(1.0),
  last_timestamp_(-1.0), point_cloud_time_diff_limit_(0.1) {}

LoFrontend::~LoFrontend() {}

bool LoFrontend::Initialize(const ros::NodeHandle& n, bool from_log) {

  name_ = ros::names::append(n.getNamespace(), "lo_frontend");

  if (!filter_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }

  if (!odometry_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud odometry.", name_.c_str());
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

bool LoFrontend::LoadParameters(const ros::NodeHandle& n) {
  // Load update rates.
  if (!pu::Get("rate/estimate", estimate_update_rate_))
    return false;
  if (!pu::Get("rate/visualization", visualization_update_rate_))
    return false;

  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  if (!pu::Get("frame_id/base", base_frame_id_))
    return false;

  // Load Settings 
  if (!pu::Get("translation_threshold_kf", translation_threshold_kf_)){
    return false;
  }
  if (!pu::Get("point_cloud_time_diff_limit", point_cloud_time_diff_limit_)){
    return false;
  }
  

  return true;
}

bool LoFrontend::RegisterCallbacks(const ros::NodeHandle& n, bool from_log) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  visualization_update_timer_ =
      nl.createTimer(visualization_update_rate_,
                     &LoFrontend::VisualizationTimerCallback,
                     this);

  if (from_log)
    return RegisterLogCallbacks(n);
  else
    return RegisterOnlineCallbacks(n);
}

bool LoFrontend::RegisterLogCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering log callbacks.", name_.c_str());
  return CreatePublishers(n);
}

bool LoFrontend::RegisterOnlineCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering online callbacks.", name_.c_str());

  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  // Fires the timer to do the heavy work in the node.
  estimate_update_timer_ = nl.createTimer(estimate_update_rate_, &LoFrontend::EstimateTimerCallback, this);

  // TODO: Andrea: we may use tcpnodelay and put this on a separate queue.
  pcld_sub_ =
      nl.subscribe("pcld", 1, &LoFrontend::PointCloudCallback, this);

  // External attitude data providers
  imu_sub_ = nl.subscribe("IMU_TOPIC", 10000, &LoFrontend::ImuCallback, this);
  odom_sub_ = nl.subscribe("ODOM_TOPIC", 1000, &LoFrontend::OdomCallback, this); 
  pose_sub_ = nl.subscribe("POSE_TOPIC", 1000, &LoFrontend::PoseCallback, this); 

  return CreatePublishers(n);
}

bool LoFrontend::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  base_frame_pcld_pub_ =
      nl.advertise<PointCloud>("base_frame_point_cloud", 10, false);

  pose_scan_pub_ =
      nl.advertise<core_msgs::PoseAndScan>("pose_and_scan", 10, false);

  return true;
}

// ------------Trying to synchronize LIDAR and extatt readings-----------

void LoFrontend::PointCloudCallback(const PointCloud::ConstPtr& msg) {
  // ROS_INFO_STREAM("recieved pointcloud message " << msg->header.stamp);
  last_pcld_stamp_.fromNSec(
      msg->header.stamp * 1000); // todo: maybe store these in a buffer, in case
                                 // we get two pointcloud messages in the queue
  synchronizer_.AddPCLPointCloudMessage(msg);
}

void LoFrontend::ImuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  synchronizer_.AddImuMessage(msg); 
}

void LoFrontend::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  synchronizer_.AddOdomMessage(msg); 
}

void LoFrontend::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  synchronizer_.AddPoseMessage(msg); 
}

// -------------------------------------------------------------------

void LoFrontend::EstimateTimerCallback(const ros::TimerEvent& ev) {
  // Sort all messages accumulated since the last estimate update.
  synchronizer_.SortMessages(); // Andrea: Do we need it?

  int count = 0;
  float time_diff = 0;
  // Iterate through sensor messages, passing to update functions
  MeasurementSynchronizer::sensor_type type;
  unsigned int index = 0;
  while (synchronizer_.GetNextMessage(&type, &index)) {

    switch (type) {

      // Point cloud messages.
      case MeasurementSynchronizer::PCL_POINTCLOUD: {
        const MeasurementSynchronizer::Message<PointCloud>::ConstPtr& m =
            synchronizer_.GetPCLPointCloudMessage(index);
        ProcessPointCloudMessage(m->msg);
        break;
      }

      // IMU messages.
      case MeasurementSynchronizer::IMU: {
          const MeasurementSynchronizer::Message<Imu>::ConstPtr& m =
              synchronizer_.GetImuMessage(index);
          ProcessImuMessage(m->msg);
          break;
      }

      // ODOM messages.
      case MeasurementSynchronizer::ODOM: {
          const MeasurementSynchronizer::Message<Odometry>::ConstPtr& m =
              synchronizer_.GetOdomMessage(index);
          ProcessOdomMessage(m->msg);
          break;
      }

      // POSE messages.
      case MeasurementSynchronizer::POSE: {
          const MeasurementSynchronizer::Message<PoseStamped>::ConstPtr& m =
              synchronizer_.GetPoseMessage(index);
          ProcessPoseMessage(m->msg);
          break;
      }

      // Unhandled sensor messages.
      default: {
        ROS_WARN("%s: Unhandled measurement type (%s).",
                name_.c_str(),
                MeasurementSynchronizer::GetTypeString(type).c_str());
        break;
      }

    }
    count ++;
  }

  // ROS_INFO_STREAM("Number of scans processed in one LoFrontend::EstimateTimerCallback is " << count);

  // Remove processed messages from the synchronizer.
  synchronizer_.ClearMessages();
}

void LoFrontend::VisualizationTimerCallback(const ros::TimerEvent& ev) {
  mapper_.PublishMap();
}

gtsam::Pose3 LoFrontend::ToGtsam(const geometry_utils::Transform3& pose) const {
  gtsam::Vector3 t;
  t(0) = pose.translation(0);
  t(1) = pose.translation(1);
  t(2) = pose.translation(2);

  gtsam::Rot3 r(pose.rotation(0, 0),
                pose.rotation(0, 1),
                pose.rotation(0, 2),
                pose.rotation(1, 0),
                pose.rotation(1, 1),
                pose.rotation(1, 2),
                pose.rotation(2, 0),
                pose.rotation(2, 1),
                pose.rotation(2, 2));

  return gtsam::Pose3(r, t);
}

void LoFrontend::ProcessImuMessage(const Imu::ConstPtr& msg){
  odometry_.SetExternalAttitude(msg->orientation, msg->header.stamp);   
}

void LoFrontend::ProcessOdomMessage(const Odometry::ConstPtr& msg){
  odometry_.SetExternalAttitude(msg->pose.pose.orientation, msg->header.stamp);   
}

void LoFrontend::ProcessPoseMessage(const PoseStamped::ConstPtr& msg){
  odometry_.SetExternalAttitude(msg->pose.orientation, msg->header.stamp);   
}

void LoFrontend::ProcessPointCloudMessage(const PointCloud::ConstPtr& msg) {

  // Check the timestamps
  ros::Time curent_timestamp;
  curent_timestamp.fromNSec(msg->header.stamp * 1000);
  // ROS_INFO_STREAM("Current timestamp is " << curent_timestamp.toSec());
  if (last_timestamp_ > 0.0){
    // Not the first point cloud
    double delta_time = curent_timestamp.toSec() - last_timestamp_;
    ROS_INFO_STREAM("Delta time is " << delta_time);

    if (delta_time > point_cloud_time_diff_limit_){
      ROS_WARN_STREAM("Time diff between point clouds is " << delta_time << " s, which is more than the limt, " << point_cloud_time_diff_limit_ << "s [LoFrontend]. Last received to put in buffer is at " << last_pcld_stamp_ << " s.");
    }
  }

  last_timestamp_ = curent_timestamp.toSec();

  // Filter the incoming point cloud message.
  PointCloud::Ptr msg_filtered(new PointCloud);
  filter_.Filter(msg, msg_filtered);

  PointCloud::Ptr msg_transformed(new PointCloud);

  // Update odometry by performing ICP.
  if (!odometry_.UpdateEstimate(*msg_filtered)) {
    b_add_first_scan_to_key_ = true;
  }

  if (b_add_first_scan_to_key_) {
    // First update ever.
    // Transforming msg to fixed frame for non-zero initial position
    localization_.TransformPointsToFixedFrame(*msg_filtered,
                                              msg_transformed.get());
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(msg_transformed, unused.get());

    // Publish localization pose messages
    ros::Time stamp = pcl_conversions::fromPCL(msg->header.stamp);
    localization_.UpdateTimestamp(stamp);
    localization_.PublishPoseNoUpdate();

    // Revert the first scan to map
    b_add_first_scan_to_key_ = false;

    // update stored pose 
    last_keyframe_pose_ = localization_.GetIntegratedEstimate();
    return;
  }
   
  // Containers.
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
  localization_.TransformPointsToSensorFrame(*msg_neighbors,
                                             msg_neighbors.get());

  // Localize to the map. Localization will output a pointcloud aligned in the
  // sensor frame.
  localization_.MeasurementUpdate(msg_filtered, msg_neighbors, msg_base.get());

  geometry_utils::Transform3 current_pose = localization_.GetIntegratedEstimate();

  // If last translation from ICP is larger than 1m
  if (ToGtsam(geometry_utils::PoseDelta(last_keyframe_pose_, current_pose))
          .translation()
          .norm() > translation_threshold_kf_) {
    // Set identity as TransformPointsToFixedFrame adds integrated to incrememntal to transform (normally goes before the update)
    localization_.MotionUpdate(gu::Transform3::Identity());

    // Transform point cloud to update the map
    localization_.TransformPointsToFixedFrame(*msg, msg_fixed.get());
    PointCloud::Ptr unused(new PointCloud);
    mapper_.InsertPoints(msg_fixed, unused.get());
    // ROS_WARN("Updating the map");

    // Update the last keyframe pose
    last_keyframe_pose_ = current_pose;
  }

  // Publish the incoming point cloud message from the base frame.
  if (base_frame_pcld_pub_.getNumSubscribers() != 0) {
    PointCloud base_frame_pcld = *msg;
    base_frame_pcld.header.frame_id = base_frame_id_;
    base_frame_pcld_pub_.publish(base_frame_pcld);
  }

  if (pose_scan_pub_.getNumSubscribers() != 0) {
    ROS_INFO("Publishing pose and scan");
    // Send pose and scan pair
    core_msgs::PoseAndScan poseScanMsg;

    sensor_msgs::PointCloud2 scan_msg;

    pcl::toROSMsg(*msg.get(), scan_msg);

    // fromPCL()
    // toPCL()

    poseScanMsg.header.stamp = scan_msg.header.stamp;
    poseScanMsg.header.frame_id = base_frame_id_;
    poseScanMsg.scan = scan_msg;
    poseScanMsg.pose.pose = geometry_utils::ros::ToRosPose(current_pose);
    poseScanMsg.pose.header.stamp = scan_msg.header.stamp;
    poseScanMsg.pose.header.frame_id = fixed_frame_id_;

    pose_scan_pub_.publish(poseScanMsg);
  }
}