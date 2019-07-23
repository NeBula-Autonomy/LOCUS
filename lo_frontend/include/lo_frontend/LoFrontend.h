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

#ifndef LO_FRONTEND_LO_FRONTEND_H
#define LO_FRONTEND_LO_FRONTEND_H

// Std libs.
#include <math.h>

// ROS.
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <core_msgs/PoseAndScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Time.h>

// BLAM
#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>
#include <measurement_synchronizer/MeasurementSynchronizer.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_odometry/PointCloudOdometry.h>
//#include <point_cloud_localization/PointCloudLocalization.h>
//#include <point_cloud_mapper/PointCloudMapper.h>

// GTSAM
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>

class LoFrontend {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  LoFrontend();
  ~LoFrontend();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  // The from_log argument specifies whether to run SLAM online (subscribe to
  // topics) or by loading messages from a bag file.
  bool Initialize(const ros::NodeHandle& n, bool from_log);

  // Sensor message processing.
  void ProcessPointCloudMessage(const PointCloud::ConstPtr& msg);

private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
  bool RegisterLogCallbacks(const ros::NodeHandle& n);
  bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  // Sensor callbacks.
  void PointCloudCallback(const PointCloud::ConstPtr& msg);

  // Timer callbacks.
  // Will run ICP and add PC on local map.
  void EstimateTimerCallback(const ros::TimerEvent& ev);
  // TODO: Andrea: ?
  void VisualizationTimerCallback(const ros::TimerEvent& ev);

  gtsam::Pose3 ToGtsam(const geometry_utils::Transform3& pose) const;

  // position when points were last added to map
  geometry_utils::Transform3 last_keyframe_; // TODO: Andrea: check if used
  ros::Time last_pcld_stamp_;                // TODO: Andrea: check if used.

  // The node's name.
  std::string name_;

  // Update rates and callback timers.
  double estimate_update_rate_;
  double visualization_update_rate_;
  ros::Timer estimate_update_timer_;
  ros::Timer visualization_update_timer_;

  // Subscribers.
  ros::Subscriber pcld_sub_; // pc from lidar

  // Publishers
  ros::Publisher base_frame_pcld_pub_; // TODO: Andrea: ?
  ros::Publisher pose_scan_pub_;       // pose and scan ?

  // Names of coordinate frames.
  std::string fixed_frame_id_; // TODO: Andrea: ?
  std::string base_frame_id_;  // TODO: Andrea: ?

  // Class objects (LoFrontend is a composite class).
  MeasurementSynchronizer synchronizer_;
  PointCloudFilter filter_;
  PointCloudOdometry odometry_;
  //PointCloudLocalization localization_; // TODO: Andrea: needed?
  //PointCloudMapper mapper_;             // TODO: Andrea: needed?
};

#endif // LO_FRONTEND_LO_FRONTEND_H