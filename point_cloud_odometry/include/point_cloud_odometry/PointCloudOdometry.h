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

#ifndef POINT_CLOUD_ODOMETRY_H
#define POINT_CLOUD_ODOMETRY_H

#include <geometry_utils/Transform3.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Float64.h>

class PointCloudOdometry {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  PointCloudOdometry();
  ~PointCloudOdometry();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n);

  // Align incoming point cloud with previous point cloud, updating odometry.
  bool UpdateEstimate(const PointCloud& points);

  // Get pose estimates.
  const geometry_utils::Transform3& GetIncrementalEstimate() const;
  const geometry_utils::Transform3& GetIntegratedEstimate() const;

  // Get the most recent point cloud that we processed. Return false and warn if
  // not initialized.
  bool GetLastPointCloud(PointCloud::Ptr& out) const;

  // Pose estimates.
  geometry_utils::Transform3 integrated_estimate_;
  geometry_utils::Transform3 incremental_estimate_;

  // Enables external attitude data fusion
  void SetExternalAttitude(const geometry_msgs::Quaternion_<std::allocator<void>>& quaternion, const ros::Time& timestamp); 


private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Use ICP between a query and reference point cloud to estimate pose.
  bool UpdateICP();

  // Publish reference and query point clouds.
  void PublishPoints(const PointCloud::Ptr& points, const ros::Publisher& pub);

  // Publish incremental and integrated pose estimates.
  void PublishPose(const geometry_utils::Transform3& pose,
                   const ros::Publisher& pub);

  // IMU Debug section 

  void PublishRpyExtatt(const geometry_msgs::Vector3& rpy,
                     const ros::Publisher& pub);

  void PublishRpyComputed(const geometry_msgs::Vector3& rpy,
                       const ros::Publisher& pub); 

  void PublishTimestampDifference(const std_msgs::Float64& timediff,
                                  const ros::Publisher& pub);

  // Subscribe to odometry from external estimator too be used as prior.
  void StateEstimateOdometryCallback(const nav_msgs::Odometry& msg);

  // The node's name.
  std::string name_;

  // External Attitude Data
  Eigen::Quaternionf extatt_first_attitude_, extatt_current_attitude_, extatt_previous_attitude_, extatt_change_in_attitude_; 
  bool use_extatt_data_, extatt_data_has_been_received_, check_extatt_data_ ; 
  float extatt_threshold_;
  struct extatt_data {
    Eigen::Quaternionf internal_extatt_attitude_; // Quaternionf
    ros::Time internal_extatt_attitude_timestamp_;
  };
  std::deque<extatt_data> extatt_deque_;
  std::deque<Eigen::Quaternionf> extatt_attitude_deque_;

  // External Attitude Data queue sizes 
  static constexpr size_t max_extatt_deque_size_ = 100; 
  static constexpr size_t min_extatt_deque_size_ = 98; 


  // Account for integrateded estimate
  double integrated_roll_, integrated_pitch_, integrated_yaw_; 

  // Publishers.
  ros::Publisher reference_pub_;
  ros::Publisher query_pub_;
  ros::Publisher incremental_estimate_pub_;
  ros::Publisher integrated_estimate_pub_;
  ros::Publisher rpy_extatt_pub_;
  ros::Publisher rpy_computed_pub_;
  ros::Publisher timestamp_difference_pub_;

  // Subscribers
  ros::Subscriber
      state_estimator_sub_; // State estimate from an external estimator (such
                            // as LION) used as prior.

  // Most recent point cloud time stamp for publishers.
  ros::Time stamp_;

  // Coordinate frames.
  std::string fixed_frame_id_;
  std::string odometry_frame_id_;

  // Transform broadcasting to other nodes.
  tf2_ros::TransformBroadcaster tfbr_;

  // For initialization.
  bool initialized_;

  // Point cloud containers.
  PointCloud::Ptr query_;
  PointCloud::Ptr reference_;

  // Parameters for filtering, and ICP.
  struct Parameters {
    // Stop ICP if the transformation from the last iteration was this small.
    double icp_tf_epsilon;

    // During ICP, two points won't be considered a correspondence if they are
    // at least this far from one another.
    double icp_corr_dist;

    // Iterate ICP this many times.
    unsigned int icp_iterations;
  } params_;

  // Maximum acceptable translation and rotation tolerances. If
  // transform_thresholding_ is set to false, neither of these thresholds are
  // considered.
  bool transform_thresholding_;
  double max_translation_;
  double max_rotation_;
};

#endif
