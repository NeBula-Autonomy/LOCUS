/*
Authors:
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
  - Andrea Tagliabue  (andrea.tagliabue@jpl.nasa.gov)
  - Andrzej Reinke (andrzej.m.reinke@jpl.nasa.gov)
*/

#include <chrono>
#include <point_cloud_localization/PointCloudLocalization.h>
#include <point_cloud_localization/utils.h>
#include <tf/transform_datatypes.h>

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

  if (!pu::Get("localization/registration_method", params_.registration_method))
    return false;
  if (!pu::Get("localization/compute_icp_covariance",
               params_.compute_icp_covariance))
    return false;
  if (!pu::Get("localization/icp_covariance_method",
               params_.icp_covariance_method))
    return false;
  if (!pu::Get("localization/icp_max_covariance", params_.icp_max_covariance))
    return false;
  if (!pu::Get("localization/compute_icp_observability",
               params_.compute_icp_observability))
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
  if (!pu::Get("localization/enable_timing_output",
               params_.enable_timing_output))
    return false;
  if (!pu::Get("localization/max_translation", max_translation_))
    return false;
  if (!pu::Get("localization/max_rotation", max_rotation_))
    return false;
  if (!pu::Get("localization/normal_search_radius",
               params_.k_nearest_neighbours_))
    return false;
  if (!pu::Get("b_is_flat_ground_assumption", b_is_flat_ground_assumption_))
    return false;
  if (!pu::Get("localization/recompute_covariance_local_map",
               recompute_covariance_local_map_))
    return false;

  if (!pu::Get("localization/recompute_covariance_scan",
               recompute_covariance_scan_))
    return false;

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
    integrated_estimate_.rotation =
        gu::Rot3(0, 0, integrated_estimate_.rotation.Yaw());
  }

  if (!b_have_fiducial) {
    ROS_WARN("Can't find fiducials, using origin");
  } else {
    ROS_INFO_STREAM("Using:\n" << integrated_estimate_);
  }

  return true;
}

bool PointCloudLocalization::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  query_pub_ =
      nl.advertise<PointCloudF>("localization_query_points", 10, false);
  reference_pub_ =
      nl.advertise<PointCloudF>("localization_reference_points", 10, false);
  aligned_pub_ =
      nl.advertise<PointCloudF>("localization_aligned_points", 10, false);
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
  odometry_pub_ = nl.advertise<nav_msgs::Odometry>("odometry", 10, false);
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
}

bool PointCloudLocalization::MotionUpdate(
    const gu::Transform3& incremental_odom) {
  // Store the incremental transform from odometry
  incremental_estimate_ = incremental_odom;
  return true;
}

bool PointCloudLocalization::TransformPointsToFixedFrame(
    const PointCloudF& points, PointCloudF* points_transformed) const {
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

  pcl::transformPointCloudWithNormals(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::TransformPointsToSensorFrame(
    const PointCloudF& points, PointCloudF* points_transformed) const {
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

  pcl::transformPointCloudWithNormals(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::SetupICP() {
  ROS_INFO("PointCloudLocalization - SetupICP");
  switch (getRegistrationMethodFromString(params_.registration_method)) {
  case RegistrationMethod::GICP: {
    ROS_INFO_STREAM("RegistrationMethod::GICP activated.");
    pcl::MultithreadedGeneralizedIterativeClosestPoint<PointF, PointF>::Ptr
        gicp = boost::make_shared<
            pcl::MultithreadedGeneralizedIterativeClosestPoint<PointF,
                                                               PointF>>();

    gicp->setTransformationEpsilon(params_.tf_epsilon);
    gicp->setMaxCorrespondenceDistance(params_.corr_dist);
    gicp->setMaximumIterations(params_.iterations);
    gicp->setRANSACIterations(0);
    gicp->setMaximumOptimizerIterations(50);
    gicp->setNumThreads(params_.num_threads);
    gicp->enableTimingOutput(params_.enable_timing_output);
    gicp->RecomputeTargetCovariance(recompute_covariance_local_map_);
    gicp->RecomputeSourceCovariance(
        recompute_covariance_scan_); // local scan we don't need to recompute
    icp_ = gicp;
    break;
  }
  case RegistrationMethod::NDT: {
    ROS_INFO_STREAM("RegistrationMethod::NDT activated.");
    pclomp::NormalDistributionsTransform<PointF, PointF>::Ptr ndt_omp =
        boost::make_shared<
            pclomp::NormalDistributionsTransform<PointF, PointF>>();

    ndt_omp->setTransformationEpsilon(params_.tf_epsilon);
    ndt_omp->setMaxCorrespondenceDistance(params_.corr_dist);
    ndt_omp->setMaximumIterations(params_.iterations);
    ndt_omp->setRANSACIterations(0);
    ndt_omp->setNumThreads(params_.num_threads);
    ndt_omp->enableTimingOutput(params_.enable_timing_output);
    icp_ = ndt_omp;
    break;
  }
  default:
    throw std::runtime_error(
        "No such Registration mode or not implemented yet " +
        params_.registration_method);
  }
  return true;
}

bool PointCloudLocalization::MeasurementUpdate(
    const PointCloudF::Ptr& query,
    const PointCloudF::Ptr& reference,
    PointCloudF* aligned_query) {
  if (aligned_query == NULL) {
    ROS_ERROR("%s: Output is null.", name_.c_str());
    is_healthy_ = false;
    return false;
  }

  // Store time stamp
  stamp_.fromNSec(query->header.stamp * 1e3);

  icp_->setInputSource(query);
  icp_->setInputTarget(reference);

  PointCloudF icpAlignedPointsLocalization_;
  icp_->align(icpAlignedPointsLocalization_);

  // Retrieve transformation and estimate and update
  const Eigen::Matrix4f T = icp_->getFinalTransformation();
  pcl::transformPointCloudWithNormals(*query, *aligned_query, T);

  KdTree::Ptr search_tree_ = icp_->getSearchMethodTarget();
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
    case (0): {
      ROS_ERROR_STREAM("Since this method wasn't used but it demanded fitness "
                       "score computation we removed it. For backup see: "
                       "110dc0df7e6fa5557b8d373222582bb9047c3254");
      EXIT_FAILURE;
      break;
    }
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
    const PointCloudF& query_cloud,
    const PointCloudF& reference_cloud,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>* eigenvectors_ptr,
    Eigen::Matrix<double, 6, 1>* eigenvalues_ptr,
    Eigen::Matrix<double, 6, 6>* A_ptr) {
  // Get normals
  // PointNormal::Ptr reference_normals(new PointNormal); // pc with normals
  PointCloudF::Ptr query_normalized(new PointCloudF); // pc whose points have
                                                      // been rearranged.
  // addNormal(reference_cloud, reference_normals,
  // params_.k_nearest_neighbours_);
  normalizePCloud(query_cloud, query_normalized);

  // Check input pointers not null
  if (not eigenvectors_ptr or not eigenvalues_ptr) {
    ROS_ERROR("Null pointer error in ComputIcpObservability");
  }
  // auto start = std::chrono::high_resolution_clock::now();

  Eigen::Matrix<double, 6, 6> Ap;
  // Compute Ap and its eigengectors
  ComputeAp_ForPoint2PlaneICP(
      query_normalized, reference_cloud, correspondences, T, Ap);
  doEigenDecomp6x6(Ap, *eigenvalues_ptr, *eigenvectors_ptr);
  *A_ptr = Ap;
}

bool PointCloudLocalization::ComputePoint2PlaneICPCovariance(
    const PointCloudF& query_cloud,
    const PointCloudF& reference_cloud,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>* covariance) {
  // Get normals
  // PointNormal::Ptr reference_normals(new PointNormal); // pc with normals
  PointCloudF::Ptr query_normalized(new PointCloudF); // pc whose points have
                                                      // been rearranged.
  Eigen::Matrix<double, 6, 6> Ap;

  // addNormal(reference_cloud, reference_normals,
  // params_.k_nearest_neighbours_);
  normalizePCloud(query_cloud, query_normalized);
  ComputeAp_ForPoint2PlaneICP(
      query_normalized, reference_cloud, correspondences, T, Ap);
  // 1 cm covariance for now hard coded
  *covariance = 0.01 * 0.01 * Ap.inverse();

  // Here bound the covariance using eigen values
  //// First find ldlt decomposition
  auto ldlt = covariance->ldlt();
  Eigen::MatrixXd L = ldlt.matrixL();
  Eigen::VectorXd vecD = ldlt.vectorD();

  double lower_bound = 1e-12;
  double upper_bound = params_.icp_max_covariance;

  bool recompute = false;
  for (size_t i = 0; i < vecD.size(); i++) {
    if (vecD(i) <= 0) {
      vecD(i) = lower_bound;
      recompute = true;
    }
    if (vecD(i) > upper_bound) {
      vecD(i) = upper_bound;
      recompute = true;
    }
  }

  if (recompute)
    *covariance = L * vecD.asDiagonal() * L.transpose();

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

  PublishPose(
      incremental_estimate_, icp_covariance_, incremental_estimate_pub_);
  PublishPose(integrated_estimate_, icp_covariance_, integrated_estimate_pub_);
  PublishOdometry(integrated_estimate_, icp_covariance_);
}

void PointCloudLocalization::PublishPose(
    const geometry_utils::Transform3& pose,
    const Eigen::Matrix<double, 6, 6>& covariance,
    const ros::Publisher& pub) {
  // Check for subscribers before doing any work
  if (pub.getNumSubscribers() == 0)
    return;

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
        .maxCoeff(&maxRow_z, &maxCol_z); // Find which one is z

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
          eigenvectors3(2,
                        i)); // LAST three coordinates = translation coordinates

      int sign = (direction(1) > 0) ? 1 : -1;
      direction = sign * direction; // To keep consistency when plotting (v and
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
    const PointCloudF::Ptr query_normalized,
    const PointNormal::Ptr reference_normals,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>& Ap) {
  Ap = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> A_i = Eigen::Matrix<double, 6, 6>::Zero();

  Eigen::Vector3d a_i, n_i;
  for (uint32_t i = 0; i < query_normalized->size(); i++) {
    a_i << query_normalized->points[i].x, //////
        query_normalized->points[i].y,    //////
        query_normalized->points[i].z;

    n_i << reference_normals->points[correspondences[i]].normal_x, //////
        reference_normals->points[correspondences[i]].normal_y,    //////
        reference_normals->points[correspondences[i]].normal_z;

    if (a_i.hasNaN() || n_i.hasNaN())
      continue;

    Eigen::Matrix<double, 1, 6> H = Eigen::Matrix<double, 1, 6>::Zero();
    H.block(0, 0, 1, 3) = (a_i.cross(n_i)).transpose();
    H.block(0, 3, 1, 3) = n_i.transpose();
    Ap += H.transpose() * H;
  }
}
void PointCloudLocalization::ComputeAp_ForPoint2PlaneICP(
    const PointCloudF::Ptr query_normalized,
    const PointCloudF& reference_normals,
    const std::vector<size_t>& correspondences,
    const Eigen::Matrix4f& T,
    Eigen::Matrix<double, 6, 6>& Ap) {
  Ap = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> A_i = Eigen::Matrix<double, 6, 6>::Zero();

  Eigen::Vector3d a_i, n_i;
  for (uint32_t i = 0; i < query_normalized->size(); i++) {
    a_i << query_normalized->points[i].x, //////
        query_normalized->points[i].y,    //////
        query_normalized->points[i].z;

    n_i << reference_normals.points[correspondences[i]].normal_x, //////
        reference_normals.points[correspondences[i]].normal_y,    //////
        reference_normals.points[correspondences[i]].normal_z;

    if (a_i.hasNaN() || n_i.hasNaN())
      continue;

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
  odometry_msg.pose.pose.orientation =
      gr::ToRosQuat(gu::RToQuat(odometry.rotation));
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

  if (is_healthy_) {
    diag_status.level = 0; // OK
    diag_status.message = "Healthy";
  } else {
    diag_status.level = 2; // ERROR
    diag_status.message = "Non healthy - Null output in MeasurementUpdate.";
  }

  return diag_status;
}
