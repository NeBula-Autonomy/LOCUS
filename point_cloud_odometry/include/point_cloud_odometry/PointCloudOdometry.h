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
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
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

  // Publish timestamp difference between external attitude and LIDAR signal 
  void PublishTimestampDifference(const std_msgs::Float64& timediff,
                                  const ros::Publisher& pub);

  bool LoadCalibrationFromTfTree();

  // The node's name.
  std::string name_;

  // External Attitude Data
  Eigen::Quaterniond external_attitude_first_, external_attitude_current_, external_attitude_previous_, external_attitude_change_; 
  bool use_external_attitude_, external_attitude_has_been_received_; 
  struct external_attitude {
    Eigen::Quaterniond internal_external_attitude_;
    ros::Time internal_external_attitude_timestamp_;
  };
  std::deque<external_attitude> external_attitude_deque_;
  std::deque<Eigen::Quaterniond> external_attitude_change_deque_;
  static constexpr size_t max_external_attitude_deque_size_ = 100; 
  static constexpr size_t min_external_attitude_deque_size_ = 99; 

  // Publishers.
  ros::Publisher reference_pub_;
  ros::Publisher query_pub_;
  ros::Publisher incremental_estimate_pub_;
  ros::Publisher integrated_estimate_pub_;
  ros::Publisher timestamp_difference_pub_;

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

  /*
  --------- Adding support for IMU Lidar calibration ------- 
  */
  std::string base_frame_id_; 
  std::string imu_frame_id_;  

  tf::TransformListener imu_T_laser_listener_;
  Eigen::Affine3d I_T_B_;    
  Eigen::Affine3d B_T_I_; 
  Eigen::Quaterniond I_T_B_q_;     

  /*
  ------------- Deactivate external attitude usage when external provider crashes at start -------------
  
  DOCUMENTATION:  This counter keeps track of how many time UpdateEstimate has been called
                  and deactivate external attitude usage after a value of 25 is reached
                  enabling the code to not being stuck in a uninitialized state and continue by relying on pure ICP Lidar 
  */
  int number_of_calls_;   

};

#endif