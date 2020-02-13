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

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

using pcl::copyPointCloud;
using pcl::PointCloud;
using pcl::PointXYZI;

PointCloudOdometry::PointCloudOdometry() : 
  initialized_(false), 
  b_use_imu_integration_(false), 
  b_use_odometry_integration_(false), 
  b_use_pose_stamped_integration_(false) {
  query_.reset(new PointCloud);
  reference_.reset(new PointCloud);
}

PointCloudOdometry::~PointCloudOdometry() {}

bool PointCloudOdometry::Initialize(const ros::NodeHandle& n) {
  ROS_INFO("PointCloudOdometry - Initialize");
  name_ = ros::names::append(n.getNamespace(), "PointCloudOdometry");
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  if(!SetupICP()){
    ROS_ERROR("Failed to SetupICP");
  }
  return true;
}

bool PointCloudOdometry::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("PointCloudOdometry - LoadParameters");

  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  if (!pu::Get("frame_id/odometry", odometry_frame_id_))
    return false;

  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_qx = 0.0, init_qy = 0.0, init_qz = 0.0, init_qw = 1.0;
  bool b_have_fiducial = true;

  if (!pu::Get("fiducial_calibration/position/x", init_x))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/position/y", init_y))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/position/z", init_z))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/x", init_qx))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/y", init_qy))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/z", init_qz))
    b_have_fiducial = false;
  if (!pu::Get("fiducial_calibration/orientation/w", init_qw))
    b_have_fiducial = false;

  double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  gu::Quat q(gu::Quat(init_qw, init_qx, init_qy, init_qz));
  gu::Rot3 m1;
  m1 = gu::QuatToR(q);
  init_roll = m1.Roll();
  init_pitch = m1.Pitch();
  init_yaw = m1.Yaw();

  gu::Transform3 init;
  init.translation = gu::Vec3(init_x, init_y, init_z);
  init.rotation = gu::Rot3(init_roll, init_pitch, init_yaw);
  integrated_estimate_ = init;

  if (!b_have_fiducial) {
    ROS_WARN("Can't find fiducials, using origin");
  } 
  else {
    ROS_INFO_STREAM("Have loaded fiducial pose, using:\n" << init);
  }

  if (!pu::Get("icp/tf_epsilon", params_.icp_tf_epsilon))
    return false;
  if (!pu::Get("icp/corr_dist", params_.icp_corr_dist))
    return false;
  if (!pu::Get("icp/iterations", params_.icp_iterations))
    return false;
  if (!pu::Get("icp/transform_thresholding", transform_thresholding_))
    return false;
  if (!pu::Get("icp/max_translation", max_translation_))
    return false;
  if (!pu::Get("icp/max_rotation", max_rotation_))
    return false;
  if (!pu::Get("icp/num_threads", params_.num_threads)) 
    return false;
  if (!pu::Get("icp/enable_timing_output", params_.enable_timing_output)) 
    return false;
  if (!pu::Get("b_verbose", b_verbose_))
    return false;
  if (!pu::Get("b_is_flat_ground_assumption", b_is_flat_ground_assumption_))
    return false;
   
  return true;
}

bool PointCloudOdometry::RegisterCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("PointCloudOdometry - RegisterCallbacks");
  ros::NodeHandle nl(n);
  query_pub_ = nl.advertise<PointCloud>("odometry_query_points", 10, false);
  reference_pub_ = nl.advertise<PointCloud>("odometry_reference_points", 10, false);
  incremental_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>("odometry_incremental_estimate", 10, false);
  integrated_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>("odometry_integrated_estimate", 10, false);
  return true;
}

bool PointCloudOdometry::SetupICP() {
  ROS_INFO("PointCloudOdometry - SetupICP");
  icp_.setTransformationEpsilon(params_.icp_tf_epsilon);
  icp_.setMaxCorrespondenceDistance(params_.icp_corr_dist);
  icp_.setMaximumIterations(params_.icp_iterations);
  icp_.setRANSACIterations(0);
  icp_.setNumThreads(params_.num_threads);
  icp_.enableTimingOutput(params_.enable_timing_output);
  return true;
}

void PointCloudOdometry::EnableImuIntegration() {
  b_use_imu_integration_ = true;
}

void PointCloudOdometry::EnableOdometryIntegration() {
  b_use_odometry_integration_ = true;
}

void PointCloudOdometry::EnablePoseStampedIntegration() {
  b_use_pose_stamped_integration_ = true;
}

void PointCloudOdometry::DisableImuIntegration() {
  b_use_imu_integration_ = false;
}

void PointCloudOdometry::DisableOdometryIntegration() {
  b_use_odometry_integration_ = false;
}

void PointCloudOdometry::DisablePoseStampedIntegration() {
  b_use_pose_stamped_integration_ = false;
}

bool PointCloudOdometry::SetLidar(const PointCloud& points) {
  stamp_.fromNSec(points.header.stamp * 1e3);
  points_ = points;
  return true;
}

bool PointCloudOdometry::SetImuDelta(const Eigen::Matrix3d& imu_delta) {
  imu_delta_ = imu_delta;
  return true;
}

bool PointCloudOdometry::SetOdometryDelta(const tf::Transform& odometry_delta) {
  odometry_delta_ = odometry_delta;
  return true;
}

bool PointCloudOdometry::SetPoseStampedDelta(const tf::Transform& pose_stamped_delta) {
  pose_stamped_delta_ = pose_stamped_delta;
  return true;
}

bool PointCloudOdometry::UpdateEstimate() {
  if (!initialized_) {
    copyPointCloud(points_, *query_);
    initialized_ = true;
    return false;
  }
  else {
    copyPointCloud(*query_, *reference_);
    copyPointCloud(points_, *query_); 
    return UpdateICP();
  } 
} 

bool PointCloudOdometry::UpdateICP() {

  PointCloud::Ptr query_trans( new PointCloud);
  
  Eigen::Matrix4d imu_prior; 
  Eigen::Matrix4d odometry_prior;
  Eigen::Matrix4d pose_stamped_prior;

  if (b_use_imu_integration_) {
    imu_prior << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    imu_prior.block(0, 0, 3, 3) = imu_delta_;   
    pcl::transformPointCloud(*query_, *query_trans, imu_prior);  
  }
  else if (b_use_odometry_integration_) {
    Eigen::Matrix4f temp;
    pcl_ros::transformAsMatrix(odometry_delta_, temp);
    odometry_prior = temp.cast <double> ();
    pcl::transformPointCloud(*query_, *query_trans, odometry_prior);
  }
  else if (b_use_pose_stamped_integration_) {
    ROS_ERROR("To be implemented - b_use_pose_stamped_integration_");
    return false;
  }  
  else {
    *query_trans = *query_;
  }

  icp_.setInputSource(query_trans);
  icp_.setInputTarget(reference_);
  icp_.align(icpAlignedPointsOdometry_);
  icpFitnessScore_ = icp_.getFitnessScore();
  Eigen::Matrix4d T; 
  T = icp_.getFinalTransformation().cast<double>();

  if (b_use_imu_integration_) { 
    T = T * imu_prior;
  }
  else if (b_use_odometry_integration_) {
    T = T * odometry_prior;
  }
  else if (b_use_pose_stamped_integration_) {
    ROS_ERROR("To be implemented - b_use_pose_stamped_integration_");
    return false;
  }
  
  if (b_is_flat_ground_assumption_) {
    tf::Matrix3x3 rotation(T(0,0),T(0,1),T(0,2),T(1,0),T(1,1),T(1,2),T(2,0),T(2,1),T(2,2));
    double roll, pitch, yaw;
    rotation.getRPY(roll, pitch, yaw);
    incremental_estimate_.translation = gu::Vec3(T(0, 3), T(1, 3), 0);
    incremental_estimate_.rotation = gu::Rot3(cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1);      
  }
  else {
    incremental_estimate_.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
    incremental_estimate_.rotation =    gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                                 T(1, 0), T(1, 1), T(1, 2),
                                                 T(2, 0), T(2, 1), T(2, 2));
  }

  if (!transform_thresholding_ ||
      (incremental_estimate_.translation.Norm() <= max_translation_ &&
       incremental_estimate_.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    integrated_estimate_ = gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  } 
  else {
    ROS_WARN(
      "%s: Discarding incremental transformation with norm (t: %lf, r: %lf)",
      name_.c_str(),
      incremental_estimate_.translation.Norm(),
      incremental_estimate_.rotation.ToEulerZYX().Norm());
  }

  PublishPose(incremental_estimate_, incremental_estimate_pub_);
  PublishPose(integrated_estimate_, integrated_estimate_pub_);
  
  return true;

}

void PointCloudOdometry::SetFlatGroundAssumptionValue(const bool& value) {
  ROS_INFO_STREAM("PointCloudOdometry - SetFlatGroundAssumptionValue - Received: " << value);
  b_is_flat_ground_assumption_ = value;
  if (value) integrated_estimate_.rotation = gu::Rot3(0, 0, integrated_estimate_.rotation.Yaw());
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
 
void PointCloudOdometry::PublishPose(const gu::Transform3& pose,
                                     const ros::Publisher& pub) {
  if (pub.getNumSubscribers() == 0) return;
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.pose = gr::ToRosPose(pose);
  ros_pose.header.frame_id = fixed_frame_id_;
  ros_pose.header.stamp = stamp_;
  pub.publish(ros_pose);
}