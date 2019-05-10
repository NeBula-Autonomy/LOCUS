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

#ifndef BLAM_SLAM_H
#define BLAM_SLAM_H

#include <ros/ros.h>

#include <blam_slam/AddFactor.h>
#include <blam_slam/RemoveFactor.h>
#include <blam_slam/SaveGraph.h>

#include <measurement_synchronizer/MeasurementSynchronizer.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_odometry/PointCloudOdometry.h>
#include <laser_loop_closure/LaserLoopClosure.h>
#include <pcl_ros/point_cloud.h>
#include <point_cloud_localization/PointCloudLocalization.h>
#include <point_cloud_mapper/PointCloudMapper.h>

#include <core_msgs/Artifact.h>

class BlamSlam {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  BlamSlam();
  ~BlamSlam();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  // The from_log argument specifies whether to run SLAM online (subscribe to
  // topics) or by loading messages from a bag file.
  bool Initialize(const ros::NodeHandle& n, bool from_log);

  // Sensor message processing.
  void ProcessPointCloudMessage(const PointCloud::ConstPtr& msg);

  int marker_id_;

 private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
  bool RegisterLogCallbacks(const ros::NodeHandle& n);
  bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  // Sensor callbacks.
  void PointCloudCallback(const PointCloud::ConstPtr& msg);
  void ArtifactCallback(const core_msgs::Artifact& msg);

  // Timer callbacks.
  void EstimateTimerCallback(const ros::TimerEvent& ev);
  void VisualizationTimerCallback(const ros::TimerEvent& ev);

  // Loop closing. Returns true if at least one loop closure was found. Also
  // output whether or not a new keyframe was added to the pose graph.
  bool HandleLoopClosures(const PointCloud::ConstPtr& scan, bool* new_keyframe);

  // Generic add Factor service - for human loop closures to start
  bool AddFactorService(blam_slam::AddFactorRequest &request,
                        blam_slam::AddFactorResponse &response);
  // Generic remove Factor service - removes edges from pose graph
  bool RemoveFactorService(blam_slam::RemoveFactorRequest &request,
                           blam_slam::RemoveFactorResponse &response);

  bool use_chordal_factor_;

  // Service to write the pose graph and all point clouds to a zip file.
  bool SaveGraphService(blam_slam::SaveGraphRequest &request,
                        blam_slam::SaveGraphResponse &response);

  // Publish Artifacts
  void PublishArtifact(const Eigen::Vector3d& W_artifact_position,
                       const core_msgs::Artifact& msg);

  // The node's name.
  std::string name_;

  // Update rates and callback timers.
  double estimate_update_rate_;
  double visualization_update_rate_;
  ros::Timer estimate_update_timer_;
  ros::Timer visualization_update_timer_;

  // Covariances
  double position_sigma_;
  double attitude_sigma_;

  // Subscribers.
  ros::Subscriber pcld_sub_;
  ros::Subscriber artifact_sub_;

  // Publishers
  ros::Publisher base_frame_pcld_pub_;
  

  // Services
  ros::ServiceServer add_factor_srv_;
  ros::ServiceServer remove_factor_srv_;
  ros::ServiceServer save_graph_srv_;

  // Names of coordinate frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;
  bool artifacts_in_global_;
  int largest_artifact_id_; 
  bool use_artifact_loop_closure_;

  // Object IDs
  std::unordered_map<std::string, gtsam::Key> artifact_id2key_hash;

  // Class objects (BlamSlam is a composite class).
  MeasurementSynchronizer synchronizer_;
  PointCloudFilter filter_;
  PointCloudOdometry odometry_;
  LaserLoopClosure loop_closure_;
  PointCloudLocalization localization_;
  PointCloudMapper mapper_;
};

#endif
