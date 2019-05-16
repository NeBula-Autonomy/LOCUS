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
#include <visualization_msgs/Marker.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

BlamSlam::BlamSlam()
  : estimate_update_rate_(0.0),
    visualization_update_rate_(0.0),
    position_sigma_(0.01),
    attitude_sigma_(0.04),
    marker_id_(0),
    tf_listener_(tf_buffer_) {}

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
  if (!pu::Get("frame_id/artifacts_in_global", artifacts_in_global_))
    return false;

  // Covariance for odom factors
  if (!pu::Get("noise/odom_position_sigma", position_sigma_)) return false;
  if (!pu::Get("noise/odom_attitude_sigma", attitude_sigma_)) return false;

  if (!pu::Get("use_chordal_factor", use_chordal_factor_))
    return false;

  std::string graph_filename;
  if (pu::Get("load_graph", graph_filename) && !graph_filename.empty()) {
    if (loop_closure_.Load(graph_filename)) {
      PointCloud::Ptr regenerated_map(new PointCloud);
      loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());
      mapper_.Reset();
      PointCloud::Ptr unused(new PointCloud);
      mapper_.InsertPoints(regenerated_map, unused.get());

      // Also reset the robot's estimated position.
      localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());

      // Publish updated map
      mapper_.PublishMap();
    } else {
      ROS_ERROR_STREAM("Failed to load graph from " << graph_filename);
    }
  }

  return true;
}

bool BlamSlam::RegisterCallbacks(const ros::NodeHandle& n, bool from_log) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  visualization_update_timer_ = nl.createTimer(
      visualization_update_rate_, &BlamSlam::VisualizationTimerCallback, this);
      
  add_factor_srv_ = nl.advertiseService("add_factor", &BlamSlam::AddFactorService, this);
  remove_factor_srv_ = nl.advertiseService("remove_factor", &BlamSlam::RemoveFactorService, this);
  save_graph_srv_ = nl.advertiseService("save_graph", &BlamSlam::SaveGraphService, this);

  restart_srv_ = nl.advertiseService("restart", &BlamSlam::RestartService, this);

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

  pcld_sub_ = nl.subscribe("pcld", 100000, &BlamSlam::PointCloudCallback, this);

  artifact_sub_ = nl.subscribe("artifact_relative", 10, &BlamSlam::ArtifactCallback, this);

  return CreatePublishers(n);
}

bool BlamSlam::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  base_frame_pcld_pub_ =
      nl.advertise<PointCloud>("base_frame_point_cloud", 10, false);
 
  artifact_pub_ = nl.advertise<core_msgs::Artifact>("artifact", 10);
  marker_pub_ = nl.advertise<visualization_msgs::Marker>("artifact_markers", 10);

  return true;
}

bool BlamSlam::AddFactorService(blam_slam::AddFactorRequest &request,
                                blam_slam::AddFactorResponse &response) {
  // TODO - bring the service creation into this node?
  if (!request.confirmed) {
    if (request.key_from == request.key_to) {
      loop_closure_.RemoveConfirmFactorVisualization();
      return true;
    } else {
      response.confirm = true;
      response.success = loop_closure_.VisualizeConfirmFactor(
        static_cast<unsigned int>(request.key_from),
        static_cast<unsigned int>(request.key_to));
      return true;
    }
  }

  response.success = loop_closure_.AddFactor(
    static_cast<unsigned int>(request.key_from),
    static_cast<unsigned int>(request.key_to),
    request.qw, request.qx, request.qy, request.qz);
  response.confirm = false;
  if (response.success){
    std::cout << "adding factor for loop closure succeeded" << std::endl;
  } else {
    std::cout << "adding factor for loop closure failed" << std::endl;
  }

  // Update the map from the loop closures
  std::cout << "Updating the map" << std::endl;
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

  mapper_.Reset();
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());

  // Also reset the robot's estimated position.
  localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());

  // Visualize the pose graph and current loop closure radius.
  loop_closure_.PublishPoseGraph();

  // Publish updated map
  mapper_.PublishMap();

  std::cout << "Updated the map" << std::endl;

  return true;
}

bool BlamSlam::RemoveFactorService(blam_slam::RemoveFactorRequest &request,
                                   blam_slam::RemoveFactorResponse &response) {
  // TODO - bring the service creation into this node?
  response.success = loop_closure_.RemoveFactor(
    static_cast<unsigned int>(request.key_from),
    static_cast<unsigned int>(request.key_to));
  if (response.success){
    std::cout << "removing factor from pose graph succeeded" << std::endl;
  }else{
    std::cout << "removing factor from pose graph failed" << std::endl;
  }

  // Update the map from the loop closures
  std::cout << "Updating the map" << std::endl;
  PointCloud::Ptr regenerated_map(new PointCloud);
  loop_closure_.GetMaximumLikelihoodPoints(regenerated_map.get());

  mapper_.Reset();
  PointCloud::Ptr unused(new PointCloud);
  mapper_.InsertPoints(regenerated_map, unused.get());

  // Also reset the robot's estimated position.
  localization_.SetIntegratedEstimate(loop_closure_.GetLastPose());

  // Visualize the pose graph and current loop closure radius.
  loop_closure_.PublishPoseGraph();

  // Publish updated map
  mapper_.PublishMap();

  std::cout << "Updated the map" << std::endl;

  return true;
}

bool BlamSlam::SaveGraphService(blam_slam::SaveGraphRequest &request,
                                blam_slam::SaveGraphResponse &response) {
  std::cout << "Saving graph..." << std::endl;
  response.success = loop_closure_.Save(request.filename);
  return true;
}

void BlamSlam::PointCloudCallback(const PointCloud::ConstPtr& msg) {
  // ROS_INFO_STREAM("Received Point Cloud " << msg->header.seq);
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

        // ROS_INFO_STREAM("Processing Point Cloud " << m->msg->header.seq);
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

void BlamSlam::ArtifactCallback(const core_msgs::Artifact& msg) {
  // Subscribe to artifact messages and include them in the posegraph

  // TODO review logic to filter - not accept if confidence is not high enough
  // if (msg.confidence < 0.5) {
  //   std::cout << "Object " << msg.id << " confidence is not sufficiently high, "
  //             << msg.confidence << " - rejecting" << std::endl;
  //   return;
  // }
  // TODO other logic - if have seen the object before within the past n
  // seconds, then don't add a new node

  // Passed - accept artifact, add new pose to pose-graph

  // Add new node - from latest relative poses from LIO
  // AddNewNode(timestamp_lkf_nsec, msg.header, false);



  std::cout << "Artifact message received is for id " << msg.id << std::endl;
  std::cout << "\t Confidence: " << msg.confidence << std::endl;
  std::cout << "\t Position:\n[" << msg.point.point.x << ", "
            << msg.point.point.y << ", " << msg.point.point.z << "]"
            << std::endl;
  std::cout << "\t Label: " << msg.label << std::endl;

  // Get artifact position 
  Eigen::Vector3d artifact_position;
  artifact_position << msg.point.point.x, msg.point.point.y, msg.point.point.z;

  // Global position
  Eigen::Vector3d W_artifact_position;

  if (artifacts_in_global_) {
    // Already in fixed frame
    W_artifact_position = artifact_position;
  } else {
    // Get global pose
    // geometry_utils::Transform3 global_pose =
    // localization_.GetIntegratedEstimate();
    geometry_utils::Transform3 global_pose =
        loop_closure_.GetPoseAtTime(msg.point.header.stamp);

    // tf::StampedTransform odom_T_base_link_tf;
    // std::string robot_name_ = "husky";
    // listener.lookupTransform(tf::resolve(robot_name_, "map"),
    // tf::resolve(robot_name_, "base_link"), msg.header.stamp,
    //                              odom_T_base_link_tf);
    // Eigen::Vector3d T_global =
    //         Eigen::Vector3d(odom_T_base_link_tf.getOrigin().getX(),
    //         odom_T_base_link_tf.getOrigin().getY(),
    //                         odom_T_base_link_tf.getOrigin().getZ());
    // Eigen::Matrix3d R_global =
    //         Eigen::Quaterniond(odom_T_base_link_tf.getRotation().w(),
    //         odom_T_base_link_tf.getRotation().x(),
    //                            odom_T_base_link_tf.getRotation().y(),
    //                            odom_T_base_link_tf.getRotation().z())
    //             .toRotationMatrix();

    // todo - check the order
    // Assum transform from body to global
    Eigen::Matrix<double, 3, 3> R_global = global_pose.rotation.Eigen();
    Eigen::Matrix<double, 3, 1> T_global = global_pose.translation.Eigen();
    std::cout << "Global robot position is: " << T_global[0] << ", "
              << T_global[1] << ", " << T_global[2] << std::endl;
    std::cout << "Global robot rotation is: " << R_global << std::endl;

    // Apply transform
    W_artifact_position = R_global * artifact_position + T_global;
  }

  std::cout << "Artifact position in map is: " << W_artifact_position[0] << ", "
            << W_artifact_position[1] << ", " << W_artifact_position[2]
            << std::endl;

  PublishArtifact(W_artifact_position, msg); 
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
  if (loop_closure_.skip_init_ == true){
    return;
    }
    else{
      loop_closure_.AddKeyScanPair(0, msg);
    return;
    }
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

bool BlamSlam::RestartService(blam_slam::RestartRequest &request,
                                blam_slam::RestartResponse &response) {
 ROS_INFO_STREAM(request.filename);
 response.success = loop_closure_.Load(request.filename);
  return true;
}

bool BlamSlam::HandleLoopClosures(const PointCloud::ConstPtr& scan,
                                  bool* new_keyframe) {
  if (new_keyframe == NULL) {
    ROS_ERROR("%s: Output boolean for new keyframe is null.", name_.c_str());
    return false;
  }

  unsigned int pose_key;
  if (!use_chordal_factor_) {
    // Add the new pose to the pose graph (BetweenFactor)
    // TODO rename to attitude and position sigma 
    gu::MatrixNxNBase<double, 6> covariance;
    covariance.Zeros();
    for (int i = 0; i < 3; ++i)
      covariance(i, i) = attitude_sigma_*attitude_sigma_; //0.4, 0.004; 0.2 m sd
    for (int i = 3; i < 6; ++i)
      covariance(i, i) = position_sigma_*position_sigma_; //0.1, 0.01; sqrt(0.01) rad sd

    const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
    if (!loop_closure_.AddBetweenFactor(localization_.GetIncrementalEstimate(),
                                        covariance, stamp, &pose_key)) {
      return false;
    }

  } else {
    // Add the new pose to the pose graph (BetweenChordalFactor)
    gu::MatrixNxNBase<double, 12> covariance;
    covariance.Zeros();
    for (int i = 0; i < 9; ++i)
      covariance(i, i) = attitude_sigma_*attitude_sigma_; //0.1, 0.01; sqrt(0.01) rad sd
    for (int i = 9; i < 12; ++i)
      covariance(i, i) = position_sigma_*position_sigma_; //0.4, 0.004; 0.2 m sd

    const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
    if (!loop_closure_.AddBetweenChordalFactor(localization_.GetIncrementalEstimate(),
                                               covariance, stamp, &pose_key)) {
      return false;
    }
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

void BlamSlam::PublishArtifact(const Eigen::Vector3d& W_artifact_position,
                               const core_msgs::Artifact& msg) {
  // Publish the message with the updated position
  core_msgs::Artifact new_msg =
      msg;              // This copying may be expensive with the thumbnail
  new_msg.header.seq++; // Change the sequence so it is not completely
                        // duplicating the message
  new_msg.point.point.x = W_artifact_position[0];
  new_msg.point.point.y = W_artifact_position[1];
  new_msg.point.point.z = W_artifact_position[2];
  new_msg.point.header.frame_id = fixed_frame_id_;

  // Transform to world frame from map frame
  new_msg.point = tf_buffer_.transform(
      new_msg.point, "world", new_msg.point.header.stamp, "world");
  // Transform at time of message

  std::cout << "Artifact position in world is: " << new_msg.point.point.x
            << ", " << new_msg.point.point.y << ", " << new_msg.point.point.z
            << std::endl;
  std::cout << "Frame ID is: " << new_msg.point.header.frame_id << std::endl;

  artifact_pub_.publish(new_msg);

  // Publish Marker with new position
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "world";
  // std::cout << "Get marker fixed frame id: " << fixed_frame_id_ << std::endl;
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "artifact";
  marker.id = marker_id_++;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position = new_msg.point.point;
  // marker.pose.position.x = W_artifact_position[0];
  // marker.pose.position.y = W_artifact_position[1];
  // marker.pose.position.z = W_artifact_position[2];
  std::cout << "Marker xyz: " << marker.pose.position.x << " "
            << marker.pose.position.y << " " << marker.pose.position.z << std::endl;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.35f;
  marker.scale.y = 0.35f;
  marker.scale.z = 0.35f;
  marker.color.a = 1.0f;

  if (msg.label == "backpack")
  {
    std::cout << "backpack marker" << std::endl;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.type = visualization_msgs::Marker::CUBE;
  }
  if (msg.label == "fire extinguisher")
  {
    std::cout << "fire extinguisher marker" << std::endl;
    marker.color.r = 1.0f;
    marker.color.g = 0.5f;
    marker.color.b = 0.75f;
    marker.type = visualization_msgs::Marker::SPHERE;
  }
  if (msg.label == "drill")
  {
    std::cout << "drill marker" << std::endl;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.type = visualization_msgs::Marker::CYLINDER;
  }
  if (msg.label == "survivor")
  {
    std::cout << "survivor marker" << std::endl;
    return;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.scale.x = 2.0f;
    marker.scale.y = 2.0f;
    marker.scale.z = 2.0f;
    marker.type = visualization_msgs::Marker::CYLINDER;
  }
  marker.lifetime = ros::Duration();

  marker_pub_.publish(marker);
}


