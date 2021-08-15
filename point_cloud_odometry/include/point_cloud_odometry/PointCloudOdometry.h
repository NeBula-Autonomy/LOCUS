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

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <eigen_conversions/eigen_msg.h>
#include <frontend_utils/CommonFunctions.h>
#include <frontend_utils/CommonStructs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <multithreaded_gicp/gicp.h>
#include <multithreaded_ndt/ndt_omp.h>
#include <nav_msgs/Odometry.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <registration_settings.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
class PointCloudOdometry {
public:
  PointCloudOdometry();
  ~PointCloudOdometry();

  bool Initialize(const ros::NodeHandle& n);

  bool SetLidar(const PointCloudF& points);
  bool SetImuDelta(const Eigen::Matrix3d& imu_delta);
  bool SetOdometryDelta(const tf::Transform& odometry_delta);
  bool SetPoseStampedDelta(const tf::Transform& pose_stamped_delta);

  bool UpdateEstimate();

  const geometry_utils::Transform3& GetIncrementalEstimate() const;
  const geometry_utils::Transform3& GetIntegratedEstimate() const;
  geometry_utils::Transform3 incremental_estimate_;
  geometry_utils::Transform3 integrated_estimate_;

  bool GetLastPointCloud(PointCloudF::Ptr& out) const;

  // Aligned point cloud returned by ICP
  PointCloudF icpAlignedPointsOdometry_;

  void EnableImuIntegration();
  void EnableOdometryIntegration();
  void EnablePoseStampedIntegration();
  void DisableSensorIntegration();

  void SetFlatGroundAssumptionValue(const bool& value);

  void PublishAll();

  // Diagnostics
  diagnostic_msgs::DiagnosticStatus GetDiagnostics();

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Use ICP between a query and reference point cloud to estimate pose
  bool UpdateICP();

  // Publish incremental and integrated pose estimates
  void PublishPose(const geometry_utils::Transform3& pose,
                   const ros::Publisher& pub);

  std::string name_;
  bool b_verbose_;
  bool initialized_;

  // Publishers
  ros::Publisher reference_pub_;
  ros::Publisher query_pub_;
  ros::Publisher incremental_estimate_pub_;
  ros::Publisher integrated_estimate_pub_;

  // Most recent point cloud time stamp for publishers
  ros::Time stamp_;

  // Coordinate frames
  std::string fixed_frame_id_;
  std::string odometry_frame_id_;

  // Point cloud containers
  PointCloudF points_;
  PointCloudF::Ptr query_;
  PointCloudF::Ptr reference_;

  // Query point cloud container

  PointCloudF::Ptr query_trans_;

  // Maximum acceptable translation and rotation tolerances
  // If transform_thresholding_ is set to false,
  // neither of these thresholds are considered
  bool transform_thresholding_;
  double max_translation_;
  double max_rotation_;

  // ICP
  struct Parameters {
    // Registration method
    std::string registration_method;
    double icp_tf_epsilon;
    double icp_corr_dist;
    unsigned int icp_iterations;
    // Number of threads GICP is allowed to use
    int num_threads;
    // Enable GICP timing information print logs
    bool enable_timing_output;

  } params_;

  pcl::Registration<PointF, PointF>::Ptr icp_;

  bool SetupICP();

  /*--------------
  Data integration
  --------------*/

  // Imu
  bool b_use_imu_integration_;
  Eigen::Matrix3d imu_delta_;

  // Odometry
  bool b_use_odometry_integration_;
  tf::Transform odometry_delta_;

  // PoseStamped
  bool b_use_pose_stamped_integration_;
  tf::Transform pose_stamped_delta_;

  /*--------------------
  Flat ground assumption
  --------------------*/

  bool b_is_flat_ground_assumption_;

  bool recompute_covariances_;

  // Diagnostics
  bool is_healthy_;

  /*--------------
  Data integration
  --------------*/
  Eigen::Matrix4d imu_prior_;
  Eigen::Matrix4d odometry_prior_;
  Eigen::Matrix4d pose_stamped_prior_;

  /*--------------------
  Making some friends
  --------------------*/

  friend class PointCloudOdometryTest;
};

#endif
