/*
Authors:
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
  - Andrea Tagliabue  (andrea.tagliabue@jpl.nasa.gov)
*/

#include <point_cloud_localization/PointCloudLocalization.h>
#include <point_cloud_localization/utils.h>
#include <tf/transform_datatypes.h>
#include <chrono>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

PointCloudLocalization::PointCloudLocalization() {}
PointCloudLocalization::~PointCloudLocalization() {}

bool PointCloudLocalization::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudLocalization");
  is_healthy_ = false;
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  if (!SetupICP()) {
    ROS_ERROR("Failed to SetupICP");
  }
  return true;
}

bool PointCloudLocalization::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("PointCloudLocalization - LoadParameters");

  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_qx = 0.0, init_qy = 0.0, init_qz = 0.0, init_qw = 1.0;
  bool b_have_fiducial = true;

  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) 
    return false;
  if (!pu::Get("frame_id/base", base_frame_id_)) 
    return false;

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

  if (!pu::Get("localization/compute_icp_covariance", params_.compute_icp_covariance))
    return false;
  if (!pu::Get("localization/icp_covariance_method", params_.icp_covariance_method))
    return false;
  if (!pu::Get("localization/icp_max_covariance", params_.icp_max_covariance))
    return false;
  if (!pu::Get("localization/compute_icp_observability", params_.compute_icp_observability))
    return false;
  if (!pu::Get("localization/tf_epsilon", params_.tf_epsilon)) 
    return false;
  if (!pu::Get("localization/corr_dist", params_.corr_dist)) 
    return false;
  if (!pu::Get("localization/iterations", params_.iterations)) 
    return false;
  if (!pu::Get("localization/transform_thresholding", transform_thresholding_))
    return false;
  if (!pu::Get("localization/num_threads", params_.num_threads)) 
    return false;
  if (!pu::Get("localization/enable_timing_output", params_.enable_timing_output))
    return false;
  if (!pu::Get("localization/max_translation", max_translation_)) 
    return false;
  if (!pu::Get("localization/max_rotation", max_rotation_)) 
    return false;
  if (!pu::Get("localization/normal_search_radius", params_.normal_radius_))
    return false;
  if (!pu::Get("b_is_flat_ground_assumption", b_is_flat_ground_assumption_))
    return false;

  pu::Get("b_publish_tfs", b_publish_tfs_);

  double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  gu::Quat q(gu::Quat(init_qw, init_qx, init_qy, init_qz));
  gu::Rot3 R;
  R = gu::QuatToR(q);
  init_roll = R.Roll();
  init_pitch = R.Pitch();
  init_yaw = R.Yaw();

  integrated_estimate_.translation = gu::Vec3(init_x, init_y, init_z);
  integrated_estimate_.rotation = gu::Rot3(init_roll, init_pitch, init_yaw);

  if (b_is_flat_ground_assumption_) {
    integrated_estimate_.rotation = gu::Rot3(0, 0, integrated_estimate_.rotation.Yaw());
  }

  if (!b_have_fiducial) {
    ROS_WARN("Can't find fiducials, using origin");
  }
  else {
    ROS_INFO_STREAM("Using:\n" << integrated_estimate_);
  }

  return true;
}

bool PointCloudLocalization::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  query_pub_ = nl.advertise<PointCloud>("localization_query_points", 10, false);
  reference_pub_ =
      nl.advertise<PointCloud>("localization_reference_points", 10, false);
  aligned_pub_ =
      nl.advertise<PointCloud>("localization_aligned_points", 10, false);
  incremental_estimate_pub_ =
      nl.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "localization_incremental_estimate", 10, false);
  integrated_estimate_pub_ =
      nl.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          "localization_integrated_estimate", 10, false);
  condition_number_pub_ =
      nl.advertise<std_msgs::Float64>("condition_number", 10, false);
  observability_viz_pub_ = nl.advertise<visualization_msgs::MarkerArray>(
      "observability_marker", 10, false);
  observability_vector_pub_ =
      nl.advertise<geometry_msgs::Vector3>("observability_vector", 10, false);
  odometry_pub_ = 
      nl.advertise<nav_msgs::Odometry>("odometry", 10, false);
  return true;
}

const gu::Transform3& PointCloudLocalization::GetIncrementalEstimate() const {
  return incremental_estimate_;
}

const gu::Transform3& PointCloudLocalization::GetIntegratedEstimate() const {
  return integrated_estimate_;
}

void PointCloudLocalization::SetIntegratedEstimate(
    const gu::Transform3& integrated_estimate) {
  integrated_estimate_ = integrated_estimate;
  // Publish transform between fixed frame and localization frame
  if (b_publish_tfs_) {
    geometry_msgs::TransformStamped tf;
    tf.transform = gr::ToRosTransform(integrated_estimate_);
    tf.header.stamp = stamp_;
    tf.header.frame_id = fixed_frame_id_;
    tf.child_frame_id = base_frame_id_;
    tfbr_.sendTransform(tf);
  }
}

bool PointCloudLocalization::MotionUpdate(
    const gu::Transform3& incremental_odom) {
  // Store the incremental transform from odometry
  incremental_estimate_ = incremental_odom;
  return true;
}

bool PointCloudLocalization::TransformPointsToFixedFrame(
    const PointCloud& points,
    PointCloud* points_transformed) const {
  if (points_transformed == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Compose current incremental estimate from odometry with the
  // integrated estimate, and transform the incoming point cloud
  const gu::Transform3 estimate =
      gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();

  Eigen::Matrix4d tf;
  tf.block(0, 0, 3, 3) = R;
  tf.block(0, 3, 3, 1) = T;

  pcl::transformPointCloud(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::TransformPointsToSensorFrame(
    const PointCloud& points,
    PointCloud* points_transformed) const {
  if (points_transformed == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Compose current incremental estimate from odometry with the
  // integrated estimate, then invert to go from world to sensor frame
  const gu::Transform3 estimate = gu::PoseInverse(
      gu::PoseUpdate(integrated_estimate_, incremental_estimate_));
  const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();

  Eigen::Matrix4d tf;
  tf.block(0, 0, 3, 3) = R;
  tf.block(0, 3, 3, 1) = T;

  pcl::transformPointCloud(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::SetupICP() {
  icp_.setTransformationEpsilon(params_.tf_epsilon);
  icp_.setMaxCorrespondenceDistance(params_.corr_dist);
  icp_.setMaximumIterations(params_.iterations);
  icp_.setRANSACIterations(0);
  icp_.setMaximumOptimizerIterations(50);
  icp_.setNumThreads(params_.num_threads);
  icp_.enableTimingOutput(params_.enable_timing_output);
  return true;
}

bool PointCloudLocalization::MeasurementUpdate(const PointCloud::Ptr& query,
                                               const PointCloud::Ptr& reference,
                                               PointCloud* aligned_query) {
  if (aligned_query == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    is_healthy_ = false;
    return false;
  }

  // Store time stamp
  stamp_.fromNSec(query->header.stamp * 1e3);

  icp_.setInputSource(query);
  icp_.setInputTarget(reference);
  PointCloud icpAlignedPointsLocalization_;
  icp_.align(icpAlignedPointsLocalization_);
  icpFitnessScore_ = icp_.getFitnessScore();

  // Retrieve transformation and estimate and update
  const Eigen::Matrix4f T = icp_.getFinalTransformation();
  pcl::transformPointCloud(*query, *aligned_query, T);

  KdTree::Ptr search_tree_ = icp_.getSearchMethodTarget();
  // Get the correspondence indices
  std::vector<size_t> correspondences;
  for (auto point : aligned_query->points) {
    std::vector<int> matched_indices;
    std::vector<float> matched_distances;
    search_tree_->nearestKSearch(point, 1, matched_indices, matched_distances);
    correspondences.push_back(matched_indices[0]);
  }

  gu::Transform3 pose_update;

  if (b_is_flat_ground_assumption_) {
    tf::Matrix3x3 rotation(T(0, 0),
                           T(0, 1),
                           T(0, 2),
                           T(1, 0),
                           T(1, 1),
                           T(1, 2),
                           T(2, 0),
                           T(2, 1),
                           T(2, 2));
    double roll, pitch, yaw;
    rotation.getRPY(roll, pitch, yaw);
    pose_update.translation = gu::Vec3(T(0, 3), T(1, 3), 0);
    pose_update.rotation =
        gu::Rot3(cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1);
  } else {
    pose_update.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
    pose_update.rotation = gu::Rot3(T(0, 0),
                                    T(0, 1),
                                    T(0, 2),
                                    T(1, 0),
                                    T(1, 1),
                                    T(1, 2),
                                    T(2, 0),
                                    T(2, 1),
                                    T(2, 2));
  }

  // Only update if the transform is small enough
  if (!transform_thresholding_ ||
      (pose_update.translation.Norm() <= max_translation_ &&
       pose_update.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    incremental_estimate_ = gu::PoseUpdate(incremental_estimate_, pose_update);
  } else {
    ROS_WARN(
        " %s: Discarding incremental transformation with norm (t: %lf, r: %lf)",
        name_.c_str(),
        pose_update.translation.Norm(),
        pose_update.rotation.ToEulerZYX().Norm());
  }

  integrated_estimate_ =
      gu::PoseUpdate(integrated_estimate_, incremental_estimate_);

  if (params_.compute_icp_observability) {
    // Compute ICP obersvability (for lidar slip detection)
    Eigen::Matrix<double, 6, 6> eigenvectors_new;
    Eigen::Matrix<double, 6, 1> eigenvalues_new;
    observability_matrix_ = Eigen::Matrix<double, 6, 6>::Zero();
    ComputeIcpObservability(*query,
                            *reference,
                            correspondences,
                            T,
                            &eigenvectors_new,
                            &eigenvalues_new,
                            &observability_matrix_);
  }

  // Compute the covariance matrix for the estimated transform
  icp_covariance_ = Eigen::Matrix<double, 6, 6>::Zero();
  if (params_.compute_icp_covariance) {
    switch (params_.icp_covariance_method) {
      case (0):
        ComputePoint2PointICPCovariance(
            icpAlignedPointsLocalization_, T, &icp_covariance_);
        break;
      case (1):
        ComputePoint2PlaneICPCovariance(
            *query, *reference, correspondences, T, &icp_covariance_);
        break;
      default:
        ROS_ERROR(
            "Unknown method for ICP covariance calculation. Check config. ");
    }
  }
  // TODO: Improve the healthy check.
  is_healthy_ = true;

  return true;
}

void PointCloudLocalization::SetFlatGroundAssumptionValue(const bool& value) {
  ROS_INFO_STREAM(
      "PointCloudLocalization - SetFlatGroundAssumptionValue - Received: "
      << value);
  b_is_flat_ground_assumption_ = value;
  if (value)
    integrated_estimate_.rotation =
        gu::Rot3(0, 0, integrated_estimate_.rotation.Yaw());
}

void PointCloudLocalization::ComputeIcpObservability(
    const PointCloud& query_cloud,
    const PointCloud& reference_cloud,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>* eigenvectors_ptr,
    Eigen::Matrix<double, 6, 1>* eigenvalues_ptr,
    Eigen::Matrix<double, 6, 6>* A_ptr) {
  // Get normals
  PointNormal::Ptr reference_normals(new PointNormal);  // pc with normals
  PointCloud::Ptr query_normalized(new PointCloud);     // pc whose points have
                                                        // been rearranged.
  addNormal(reference_cloud, reference_normals, params_.normal_radius_);
  normalizePCloud(query_cloud, query_normalized);

  // Check input pointers not null
  if (not eigenvectors_ptr or not eigenvalues_ptr) {
    ROS_ERROR("Null pointer error in ComputIcpObservability");
  }
  // auto start = std::chrono::high_resolution_clock::now();

  Eigen::Matrix<double, 6, 6> Ap;
  // Compute Ap and its eigengectors
  ComputeAp_ForPoint2PlaneICP(
      query_normalized, reference_normals, correspondences, T, Ap);
  doEigenDecomp6x6(Ap, *eigenvalues_ptr, *eigenvectors_ptr);
  *A_ptr = Ap;
}

bool PointCloudLocalization::ComputePoint2PointICPCovariance(
    const PointCloud& pointCloud,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>* covariance) {
  geometry_utils::Transform3 ICP_transformation;

  // Extract translation values from T
  double t_x = T(0, 3);
  double t_y = T(1, 3);
  double t_z = T(2, 3);

  // Extract roll, pitch and yaw from T
  ICP_transformation.rotation = gu::Rot3(T(0, 0),
                                         T(0, 1),
                                         T(0, 2),
                                         T(1, 0),
                                         T(1, 1),
                                         T(1, 2),
                                         T(2, 0),
                                         T(2, 1),
                                         T(2, 2));
  double r = ICP_transformation.rotation.Roll();
  double p = ICP_transformation.rotation.Pitch();
  double y = ICP_transformation.rotation.Yaw();

  // Symbolic expression of the Jacobian matrix
  double J11, J12, J13, J14, J15, J16, J21, J22, J23, J24, J25, J26, J31, J32,
      J33, J34, J35, J36;

  Eigen::Matrix<double, 6, 6> H;
  H = Eigen::MatrixXd::Zero(6, 6);

  // Compute the entries of Jacobian
  // Entries of Jacobian matrix are obtained from MATLAB Symbolic Toolbox
  for (size_t i = 0; i < pointCloud.points.size(); ++i) {
    double p_x = pointCloud.points[i].x;
    double p_y = pointCloud.points[i].y;
    double p_z = pointCloud.points[i].z;

    J11 = 0.0;
    J12 = -2.0 *
          (p_z * sin(p) + p_x * cos(p) * cos(y) - p_y * cos(p) * sin(y)) *
          (t_x - p_x + p_z * cos(p) - p_x * cos(y) * sin(p) +
           p_y * sin(p) * sin(y));
    J13 = 2.0 * (p_y * cos(y) * sin(p) + p_x * sin(p) * sin(y)) *
          (t_x - p_x + p_z * cos(p) - p_x * cos(y) * sin(p) +
           p_y * sin(p) * sin(y));
    J14 = 2.0 * t_x - 2.0 * p_x + 2.0 * p_z * cos(p) -
          2.0 * p_x * cos(y) * sin(p) + 2.0 * p_y * sin(p) * sin(y);
    J15 = 0.0;
    J16 = 0.0;

    J21 = 2.0 *
          (p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
           p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
           p_z * sin(p) * sin(r)) *
          (p_y - t_y + p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) +
           p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
           p_z * cos(r) * sin(p));
    J22 = -2.0 *
          (p_z * cos(p) * cos(r) - p_x * cos(r) * cos(y) * sin(p) +
           p_y * cos(r) * sin(p) * sin(y)) *
          (p_y - t_y + p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) +
           p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
           p_z * cos(r) * sin(p));
    J23 = 2.0 *
          (p_x * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
           p_y * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y))) *
          (p_y - t_y + p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) +
           p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
           p_z * cos(r) * sin(p));
    J24 = 0.0;
    J25 = 2.0 * t_y - 2.0 * p_y -
          2.0 * p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) -
          2.0 * p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) +
          2.0 * p_z * cos(r) * sin(p);
    J26 = 0.0;

    J31 = -2.0 *
          (p_x * (sin(r) * sin(y) - cos(p) * cos(r) * cos(y)) +
           p_y * (cos(y) * sin(r) + cos(p) * cos(r) * sin(y)) -
           p_z * cos(r) * sin(p)) *
          (t_z - p_z + p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
           p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
           p_z * sin(p) * sin(r));
    J32 = 2.0 *
          (p_z * cos(p) * sin(r) - p_x * cos(y) * sin(p) * sin(r) +
           p_y * sin(p) * sin(r) * sin(y)) *
          (t_z - p_z + p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
           p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
           p_z * sin(p) * sin(r));
    J33 = 2.0 *
          (p_x * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) -
           p_y * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r))) *
          (t_z - p_z + p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
           p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
           p_z * sin(p) * sin(r));
    J34 = 0.0;
    J35 = 0.0;
    J36 = 2.0 * t_z - 2.0 * p_z +
          2.0 * p_x * (cos(r) * sin(y) + cos(p) * cos(y) * sin(r)) +
          2.0 * p_y * (cos(r) * cos(y) - cos(p) * sin(r) * sin(y)) +
          2.0 * p_z * sin(p) * sin(r);

    // Form the 3X6 Jacobian matrix
    Eigen::Matrix<double, 3, 6> J;
    J << J11, J12, J13, J14, J15, J16, J21, J22, J23, J24, J25, J26, J31, J32,
        J33, J34, J35, J36;
    // Compute J'XJ (6X6) matrix and keep adding for all the points in the point
    // cloud
    H += J.transpose() * J;
  }
  *covariance = H.inverse() * icpFitnessScore_;

  // Here bound the covariance using eigen values
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
  eigensolver.compute(*covariance);
  Eigen::VectorXd eigen_values = eigensolver.eigenvalues().real();
  Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();
  double lower_bound = 0.001;     // Should be positive semidef
  double upper_bound = params_.icp_max_covariance;
  if (eigen_values.size() < 6) {
    *covariance = Eigen::MatrixXd::Identity(6, 6) * upper_bound;
    ROS_ERROR("Failed to find eigen values when computing icp covariance");
    return false;
  }
  for (size_t i = 0; i < 6; i++) {
    if (eigen_values[i] < lower_bound) eigen_values[i] = lower_bound;
    if (eigen_values[i] > upper_bound) eigen_values[i] = upper_bound;
  }
  // Update covariance matrix after bound
  *covariance =
      eigen_vectors * eigen_values.asDiagonal() * eigen_vectors.inverse();

  // Compute the SVD of the covariance matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      *covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd( covariance, Eigen::ComputeFullV |
  // Eigen::ComputeFullU);

  // Extract the singular values from SVD
  auto singular_values = svd.singularValues();
  // The covariance matrix is a symmetric matrix, so its  singular  values  are
  // the absolute values of its nonzero eigenvalues Condition number is the
  // ratio of the largest and smallest eigenvalues.
  condition_number_ = singular_values(0) / singular_values(5);

  return true;
}

bool PointCloudLocalization::ComputePoint2PlaneICPCovariance(
    const PointCloud& query_cloud,
    const PointCloud& reference_cloud,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>* covariance) {
  // Get normals
  PointNormal::Ptr reference_normals(new PointNormal);  // pc with normals
  PointCloud::Ptr query_normalized(new PointCloud);     // pc whose points have
                                                        // been rearranged.
  Eigen::Matrix<double, 6, 6> Ap;

  addNormal(reference_cloud, reference_normals, params_.normal_radius_);
  normalizePCloud(query_cloud, query_normalized);
  ComputeAp_ForPoint2PlaneICP(
      query_normalized, reference_normals, correspondences, T, Ap);
  // 1 cm covariance for now hard coded
  *covariance = 0.01 * 0.01 * Ap.inverse();

  // Here bound the covariance using eigen values
  Eigen::EigenSolver<Eigen::MatrixXd> eigensolver;
  eigensolver.compute(*covariance);
  Eigen::VectorXd eigen_values = eigensolver.eigenvalues().real();
  Eigen::MatrixXd eigen_vectors = eigensolver.eigenvectors().real();
  double lower_bound = 0.001;  // Should be positive semidef
  double upper_bound = params_.icp_max_covariance;
  if (eigen_values.size() < 6) {
    *covariance = Eigen::MatrixXd::Identity(6, 6) * upper_bound;
    ROS_ERROR("Failed to find eigen values when computing icp covariance");
    return false;
  }
  for (size_t i = 0; i < eigen_values.size(); i++) {
    if (eigen_values(i) < lower_bound) eigen_values(i) = lower_bound;
    if (eigen_values(i) > upper_bound) eigen_values(i) = upper_bound;
  }
  // Update covariance matrix after bound
  *covariance = eigen_vectors * eigen_values.asDiagonal() *
                eigen_vectors.inverse() * icpFitnessScore_;

  // Compute the SVD of the covariance matrix
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      *covariance, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Eigen::JacobiSVD<Eigen::MatrixXd> svd( Ap, Eigen::ComputeFullV |
  // Eigen::ComputeFullU);

  // Extract the singular values from SVD
  auto singular_values = svd.singularValues();
  // The covariance matrix is a symmetric matrix, so its  singular  values  are
  // the absolute values of its nonzero eigenvalues Condition number is the
  // ratio of the largest and smallest eigenvalues.
  condition_number_ = singular_values(0) / singular_values(5);

  return true;
}

void PointCloudLocalization::PublishAll() {
  if (params_.compute_icp_observability)
    PublishObservableDirections(observability_matrix_);

  if (params_.compute_icp_covariance)
    PublishConditionNumber(condition_number_, condition_number_pub_);

  PublishPose(incremental_estimate_, icp_covariance_, incremental_estimate_pub_);
  PublishPose(integrated_estimate_, icp_covariance_, integrated_estimate_pub_);
  PublishOdometry(integrated_estimate_, icp_covariance_);

  // Publish transform between fixed frame and localization frame
  if (b_publish_tfs_) {
    geometry_msgs::TransformStamped tf;
    tf.transform = gr::ToRosTransform(integrated_estimate_);
    tf.header.stamp = stamp_;
    tf.header.frame_id = fixed_frame_id_;
    tf.child_frame_id = base_frame_id_;
    tfbr_.sendTransform(tf);
  }
}

void PointCloudLocalization::PublishPose(
    const geometry_utils::Transform3& pose,
    const Eigen::Matrix<double, 6, 6>& covariance,
    const ros::Publisher& pub) {
  // Check for subscribers before doing any work
  if (pub.getNumSubscribers() == 0) return;

  // Convert from gu::Transform3 to ROS's Pose with covariance stamped type and
  // publish
  geometry_msgs::PoseWithCovarianceStamped ros_pose;
  ros_pose.pose.pose = gr::ToRosPose(pose);
  ros_pose.header.frame_id = fixed_frame_id_;
  ros_pose.header.stamp = stamp_;

  for (size_t i = 0; i < 36; i++) {
    size_t row = static_cast<size_t>(i / 6);
    size_t col = i % 6;
    ros_pose.pose.covariance[i] = covariance(row, col);
  }

  pub.publish(ros_pose);
}

void PointCloudLocalization::PublishPoseNoUpdate() {
  // Convert pose estimates to ROS format and publish
  icp_covariance_ = Eigen::MatrixXd::Zero(6, 6);
  PublishAll();
}

void PointCloudLocalization::PublishConditionNumber(double& k,
                                                    const ros::Publisher& pub) {
  // Convert condition number value to ROS format and publish
  std_msgs::Float64 condition_number;
  condition_number.data = k;
  pub.publish(condition_number);
}

void PointCloudLocalization::PublishObservableDirections(
    const Eigen::Matrix<double, 6, 6>& A) {
  if (observability_vector_pub_.getNumSubscribers() > 0 ||
      observability_viz_pub_.getNumSubscribers() > 0) {
    // For visualization, plot only translation (not coupling between rotation
    // and translation)
    Eigen::Matrix<double, 3, 1> eigenvalues3;
    Eigen::Matrix<double, 3, 3> eigenvectors3;
    Eigen::Matrix<double, 3, 3> A_translation = A.block(3, 3, 3, 3);
    doEigenDecomp3x3(A_translation, eigenvalues3, eigenvectors3);

    // Find Eigenvector with highest component in z
    Eigen::MatrixXf::Index maxRow_z, maxCol_z;
    (eigenvectors3.row(2))
        .cwiseAbs()
        .maxCoeff(&maxRow_z, &maxCol_z);  // Find which one is z

    // Eigen::Matrix<double, 3, 1> eigenvector_z = eigenvectors3.col(maxCol_z);

    visualization_msgs::MarkerArray observability_directions;
    geometry_msgs::Vector3 observability_vector;
    observability_vector.x = 0;
    observability_vector.y = 0;
    observability_vector.z = 0;

    Eigen::Matrix<double, 3, 1> eigenvalues_normalized =
        eigenvalues3 / (eigenvalues3.maxCoeff());
    for (int i = 0; i < eigenvalues3.rows(); i++) {
      double eigenvalue_normalized = eigenvalues_normalized(i);
      Eigen::Vector3d direction(
          eigenvectors3(0, i),
          eigenvectors3(1, i),
          eigenvectors3(
              2,
              i));  // LAST three coordinates = translation coordinates

      int sign = (direction(1) > 0) ? 1 : -1;
      direction = sign * direction;  // To keep consistency when plotting (v and
                                     // -v are two valids eigenvectors)

      // Ensure Eigenvector z is poiting up
      // Eigen::MatrixXf::Index maxRow, maxCol;
      // eigenvalues_normalized.maxCoeff(&maxRow, &maxCol); //Find which one is
      // z

      if (i == maxCol_z) {
        int sign_z = (direction(2) > 0) ? 1 : -1;
        direction = sign_z * direction;
      }

      visualization_msgs::Marker marker;
      // marker.header = pclptr_normals->header;
      marker.header.frame_id = base_frame_id_;
      marker.id = i + 5000;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      geometry_msgs::Point point_init, point_end;
      point_init.x = 0.0;
      point_init.y = 0.0;
      point_init.z = 0.0;
      marker.points.push_back(point_init);

      point_end.x = point_init.x + eigenvalue_normalized * direction(0) / 0.16;
      point_end.y = point_init.y + eigenvalue_normalized * direction(1) / 0.16;
      point_end.z = point_init.z + eigenvalue_normalized * direction(2) / 0.16;
      marker.points.push_back(point_end);
      marker.scale.x = 0.13;
      marker.scale.y = 0.25;
      marker.scale.z = 0;
      marker.color.r = 1;
      marker.color.g = 0;
      marker.color.b = 0;
      marker.color.a = 1.0;
      marker.header.stamp = ros::Time::now();
      observability_directions.markers.push_back(marker);

      observability_vector.x += eigenvalue_normalized * direction(0);
      observability_vector.y += eigenvalue_normalized * direction(1);
      observability_vector.z += eigenvalue_normalized * direction(2);
    }
    if (observability_viz_pub_.getNumSubscribers() > 0) {
      observability_viz_pub_.publish(observability_directions);
    }
    if (observability_vector_pub_.getNumSubscribers() > 0) {
      observability_vector_pub_.publish(observability_vector);
    }
  }
}

void PointCloudLocalization::UpdateTimestamp(ros::Time& stamp) {
  stamp_ = stamp;
}

// Compute matrix Ap= \sum_i (Hi' Hi)
// A_i = (Hi' Hi)
// H_i=[  a_i x R'n_i'   n_i' ]'
// n_i from reference point cloud
// a_i are from query point cloud
void PointCloudLocalization::ComputeAp_ForPoint2PlaneICP(
    const PointCloud::Ptr query_normalized,
    const PointNormal::Ptr reference_normals,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>& Ap) {
  Ap = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> A_i = Eigen::Matrix<double, 6, 6>::Zero();

  Eigen::Vector3d a_i, n_i;
  for (uint32_t i = 0; i < query_normalized->size(); i++) {
    a_i << query_normalized->points[i].x,  //////
        query_normalized->points[i].y,     //////
        query_normalized->points[i].z;

    n_i << reference_normals->points[correspondences[i]].normal_x,  //////
        reference_normals->points[correspondences[i]].normal_y,     //////
        reference_normals->points[correspondences[i]].normal_z;

    if (a_i.hasNaN() || n_i.hasNaN()) continue;

    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H.block(0, 0, 1, 3) = (a_i.cross(n_i)).transpose();
    H.block(0, 3, 1, 3) = n_i.transpose();
    Ap += H.transpose() * H;
  }
}

void PointCloudLocalization::PublishOdometry(
  const geometry_utils::Transform3& odometry, 
  const Eigen::Matrix<double, 6, 6>& covariance) {
  nav_msgs::Odometry odometry_msg; 
  odometry_msg.header.stamp = stamp_; 
  odometry_msg.header.frame_id = fixed_frame_id_;
  odometry_msg.pose.pose.position = gr::ToRosPoint(odometry.translation);
  odometry_msg.pose.pose.orientation = gr::ToRosQuat(gu::RToQuat(odometry.rotation));
  for (size_t i = 0; i < 36; i++) {
    size_t row = static_cast<size_t>(i / 6);
    size_t col = i % 6;
    odometry_msg.pose.covariance[i] = covariance(row, col);
  }
  odometry_pub_.publish(odometry_msg); 
}

diagnostic_msgs::DiagnosticStatus PointCloudLocalization::GetDiagnostics() {
    diagnostic_msgs::DiagnosticStatus diag_status;
    diag_status.name = name_;

    if (is_healthy_)
    {
      diag_status.level = 0; // OK
      diag_status.message = "Healthy";
    }
    else
    {
      diag_status.level = 2; // ERROR
      diag_status.message = "Non healthy - Null output in MeasurementUpdate.";
    }

    return diag_status;
}