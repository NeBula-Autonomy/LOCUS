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

#ifndef POINT_CLOUD_LOCALIZATION_H
#define POINT_CLOUD_LOCALIZATION_H

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <multithreaded_gicp/gicp.h>
#include <multithreaded_ndt/ndt_omp.h>
#include <nav_msgs/Odometry.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_ros/point_cloud.h>
#include <registration_settings.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

using pcl::PointCloud;
using pcl::PointXYZI;
using pcl::transformPointCloud;

class PointCloudLocalization {
public:
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::PointNormal> PointNormal;
  typedef pcl::search::KdTree<pcl::PointXYZI> KdTree;

  PointCloudLocalization();
  ~PointCloudLocalization();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either
  bool Initialize(const ros::NodeHandle& n);

  // Transform a point cloud from the sensor frame into the fixed frame using
  // the current best position estimate
  bool TransformPointsToFixedFrame(const PointCloud& points,
                                   PointCloud* points_transformed) const;

  // Transform a point cloud from the fixed frame into the sensor frame using
  // the current best position estimate
  bool TransformPointsToSensorFrame(const PointCloud& points,
                                    PointCloud* points_transformed) const;

  // Store incremental estimate from odometry
  bool MotionUpdate(const geometry_utils::Transform3& incremental_odom);

  // Align incoming point cloud with a reference point cloud from the map.
  // Output the query scan aligned in the localization frame
  bool MeasurementUpdate(const PointCloud::Ptr& query,
                         const PointCloud::Ptr& reference,
                         PointCloud* aligned_query);

  bool
  ComputePoint2PlaneICPCovariance(const PointCloud& query_cloud,
                                  const PointCloud& reference_cloud,
                                  const std::vector<size_t>& correspondences,
                                  const Eigen::Matrix4f& T,
                                  Eigen::Matrix<double, 6, 6>* covariance);

  // Compute observability of ICP for two pointclouds
  void ComputeIcpObservability(const PointCloud& query_cloud,
                               const PointCloud& reference_cloud,
                               const std::vector<size_t>& correspondences,
                               const Eigen::Matrix4f& T,
                               Eigen::Matrix<double, 6, 6>* eigenvectors_ptr,
                               Eigen::Matrix<double, 6, 1>* eigenvalues_ptr,
                               Eigen::Matrix<double, 6, 6>* A_ptr);

  // Get pose estimates.
  const geometry_utils::Transform3& GetIncrementalEstimate() const;
  const geometry_utils::Transform3& GetIntegratedEstimate() const;

  // Set integrated estimate. Useful for graph SLAM whenever the pose graph is
  // updated and the map is regenerated
  void
  SetIntegratedEstimate(const geometry_utils::Transform3& integrated_estimate);

  // Pose estimate
  // TODO: why do we need an interface if we have them as  a public?
  geometry_utils::Transform3 incremental_estimate_;
  geometry_utils::Transform3 integrated_estimate_;

  // Publish All
  void PublishAll();

  // Publish for first pose
  void PublishPoseNoUpdate();

  // Update timestamp
  void UpdateTimestamp(ros::Time& stamp);

  // Aligned point cloud returned by ICP
  PointCloud icpAlignedPointsLocalization_;

  void SetFlatGroundAssumptionValue(const bool& value);

  // Diagnostics
  diagnostic_msgs::DiagnosticStatus GetDiagnostics();

private:
  // Node initialization
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Publish incremental and integrated pose estimates
  void PublishPose(const geometry_utils::Transform3& pose,
                   const Eigen::Matrix<double, 6, 6>& covariance,
                   const ros::Publisher& pub);

  void PublishOdometry(const geometry_utils::Transform3& odometry,
                       const Eigen::Matrix<double, 6, 6>& covariance);

  // Publish condition number of ICP covariance matrix
  void PublishConditionNumber(double& k, const ros::Publisher& pub);

  // Publish observability direction based on ICP of two ptclds
  void PublishObservableDirections(const Eigen::Matrix<double, 6, 6>& A);

  // The node's name
  std::string name_;

  // Publishers
  ros::Publisher reference_pub_;
  ros::Publisher query_pub_;
  ros::Publisher aligned_pub_;
  ros::Publisher incremental_estimate_pub_;
  ros::Publisher integrated_estimate_pub_;
  ros::Publisher condition_number_pub_;
  ros::Publisher observability_viz_pub_;
  ros::Publisher observability_vector_pub_;

  // Most recent point cloud time stamp for publishers
  ros::Time stamp_;

  // Coordinate frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  // Transform broadcasting to other nodes
  tf2_ros::TransformBroadcaster tfbr_;

  // Parameters for filtering and ICP
  struct Parameters {
    // What registration method should be used: GICP, NDT
    std::string registration_method;
    // Compute ICP covariance and condition number
    bool compute_icp_covariance;
    // Point-to-point or Point-to-plane
    int icp_covariance_method;
    // Max boundd for icp covariance
    double icp_max_covariance;
    // Compute ICP observability
    bool compute_icp_observability;
    // Stop ICP if the transformation from the last iteration was this small
    double tf_epsilon;
    // During ICP, two points won't be considered a correspondence if they are
    // at least this far from one another
    double corr_dist;
    // Iterate ICP this many times
    unsigned int iterations;
    // Number of threads GICP is allowed to use
    int num_threads;
    // Enable GICP timing information print logs
    bool enable_timing_output;
    // Radius used when computing ptcld normals
    //    double normal_radius_;
    int k_nearest_neighbours_;
  } params_;

  // Maximum acceptable translation and rotation tolerances.
  // If transform_thresholding_ is set to false,
  // neither of these thresholds are considered
  bool transform_thresholding_;
  double max_translation_;
  double max_rotation_;
  // double max_power_;

  // ICP
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr icp_;
  bool SetupICP();

  void ComputeAp_ForPoint2PlaneICP(const PointCloud::Ptr query_normalized,
                                   const PointNormal::Ptr reference_normals,
                                   const std::vector<size_t>& correspondences,
                                   const Eigen::Matrix4f& T,
                                   Eigen::Matrix<double, 6, 6>& Ap);

  /*--------------------
  Flat ground assumption
  --------------------*/

  bool b_is_flat_ground_assumption_;

  /*--------------------
  Odometry publishment
  --------------------*/

  ros::Publisher odometry_pub_;

  // ICP covariance
  Eigen::Matrix<double, 6, 6> icp_covariance_;
  double condition_number_;

  // Observability matrix
  Eigen::Matrix<double, 6, 6> observability_matrix_;

  // Diagnostics
  bool is_healthy_;

  /*--------------------
  Making some friends
  --------------------*/
  friend class PointCloudLocalizationTest;
};

#endif
