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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <point_cloud_odometry/PointCloudOdometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/registration/gicp.h>
#include <pcl/search/impl/search.hpp>

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
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  if (!pu::Get("frame_id/odometry", odometry_frame_id_))
    return false;

  // Load initial position.
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

  if (!b_have_fiducial) {
    ROS_WARN("Can't find fiducials, using origin");
  }

  // convert initial quaternion to Roll/Pitch/Yaw
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

  // Load algorithm parameters.
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

  return true;
}

bool PointCloudOdometry::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  state_estimator_sub_ =
      nl.subscribe("hero/lion/odom",
                   10,
                   &PointCloudOdometry::StateEstimateOdometryCallback,
                   this,
                   ros::TransportHints().tcpNoDelay());

  query_pub_ = nl.advertise<PointCloud>("odometry_query_points", 10, false);
  reference_pub_ =
      nl.advertise<PointCloud>("odometry_reference_points", 10, false);
  incremental_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "odometry_incremental_estimate", 10, false);
  integrated_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "odometry_integrated_estimate", 10, false);

  return true;
}

void PointCloudOdometry::StateEstimateOdometryCallback(
    const nav_msgs::Odometry& msg) {
  // TODO: Andrea: add odometry callback.
}

bool PointCloudOdometry::UpdateEstimate(const PointCloud& points) {
  // Store input point cloud's time stamp for publishing.
  stamp_.fromNSec(points.header.stamp * 1e3);

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

  icp.align(icpAlignedPointsOdometry_);
  icpFitnessScore_ = icp.getFitnessScore();

  const Eigen::Matrix4f T = icp.getFinalTransformation();

  // Update pose estimates.
  incremental_estimate_.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  // clang-format off
  incremental_estimate_.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                            T(1, 0), T(1, 1), T(1, 2),
                                            T(2, 0), T(2, 1), T(2, 2));
  // clang-format on

  // Only update if the incremental transform is small enough.
  if (!transform_thresholding_ ||
      (incremental_estimate_.translation.Norm() <= max_translation_ &&
       incremental_estimate_.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    integrated_estimate_ =
        gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  } else {
    ROS_WARN(
        "%s: Discarding incremental transformation with norm (t: %lf, r: %lf)",
        name_.c_str(),
        incremental_estimate_.translation.Norm(),
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

bool PointCloudOdometry::ComputeICPCovariance(const pcl::PointCloud<pcl::PointXYZ> pointCloud, const Eigen::Matrix4f T, Eigen::Matrix<double, 6, 6> covariance){
  geometry_utils::Transform3 ICP_transformation;

  // Extract translation values from T
  double t_x = T(0,3);
  double t_y = T(1,3);
  double t_z = T(2,3);

  // Extract roll, pitch and yaw from T
  ICP_transformation.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                            T(1, 0), T(1, 1), T(1, 2),
                                            T(2, 0), T(2, 1), T(2, 2));
  double r = ICP_transformation.rotation.Roll();
  double p = ICP_transformation.rotation.Pitch();
  double y = ICP_transformation.rotation.Yaw();

  // Symbolic expression of the Jacobian matrix
  double J11,    J12,	   J13,   J14,  J15,    J16,
         J21,    J22,	   J23,   J24,  J25,    J26,
         J31,    J32,	   J33,   J34,  J35,    J36;
  
  Eigen::Matrix<double, 6, 6> H;
  H = Eigen::MatrixXd::Zero(6, 6);

  // Compute the entries of Jacobian
  // Entries of Jacobian matrix are obtained from MATLAB Symbolic Toolbox
  for (size_t i = 0; i < pointCloud.points.size(); ++i){
    double p_x = pointCloud.points[i].x;
    double p_y = pointCloud.points[i].y;
    double p_z = pointCloud.points[i].z;

    J11 = 0;
    J12 = p_y*cos(p)*sin(y) - p_x*cos(p)*cos(y) - p_z*sin(p);
    J13 = p_y*cos(y)*sin(p) + p_x*sin(p)*sin(y);
    J14 = 1;
    J15 = 0;
    J16 = 0;

    J21 = -p_x*(cos(r)*sin(y) + cos(p)*cos(y)*sin(r)) - p_y*(cos(r)*cos(y) - cos(p)*sin(r)*sin(y)) - p_z*sin(p)*sin(r);
    J22 = p_z*cos(p)*cos(r) - p_x*cos(r)*cos(y)*sin(p) + p_y*cos(r)*sin(p)*sin(y);
    J23 = p_y*(sin(r)*sin(y) - cos(p)*cos(r)*cos(y)) - p_x*(cos(y)*sin(r) + cos(p)*cos(r)*sin(y));
    J24 = 0;
    J25 = 1;
    J26 = 0;

    J31 = p_z*cos(r)*sin(p) - p_y*(cos(y)*sin(r) + cos(p)*cos(r)*sin(y)) - p_x*(sin(r)*sin(y) - cos(p)*cos(r)*cos(y));
    J32 = p_z*cos(p)*sin(r) - p_x*cos(y)*sin(p)*sin(r) + p_y*sin(p)*sin(r)*sin(y);
    J33 =  p_x*(cos(r)*cos(y) - cos(p)*sin(r)*sin(y)) - p_y*(cos(r)*sin(y) + cos(p)*cos(y)*sin(r));
    J34 = 0;
    J35 = 0;
    J36 = 1;
    // Form the 3X6 Jacobian matrix
    Eigen::Matrix<double, 3, 6> J;
    J << J11,    J12,	   J13,   J14,  J15,    J16,
         J21,    J22,	   J23,   J24,  J25,    J26,
         J31,    J32,	   J33,   J34,  J35,    J36;
    // Compute J'XJ (6X6) matrix and keep adding for all the points in the point cloud
    H += J.transpose() * J;
  }
  Eigen::Matrix<double, 6, 6> cov;
  cov = H.inverse() * icpFitnessScore_;
  covariance = cov;
  
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
