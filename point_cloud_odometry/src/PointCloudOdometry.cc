/*
Authors:
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
  - Andrzej Reinke    (andrzej.m.reinke@jpl.nasa.gov)
*/

#include <point_cloud_odometry/PointCloudOdometry.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

using pcl::copyPointCloud;

PointCloudOdometry::PointCloudOdometry()
  : initialized_(false),
    b_use_imu_integration_(false),
    b_use_odometry_integration_(false) {
  query_.reset(new PointCloudF);
  reference_.reset(new PointCloudF);
  query_trans_.reset(new PointCloudF);
  query_trans_->reserve(50000);
  query_trans_->clear();
}

PointCloudOdometry::~PointCloudOdometry() {}

bool PointCloudOdometry::Initialize(const ros::NodeHandle& n) {
  ROS_INFO("PointCloudOdometry - Initialize");
  name_ = ros::names::append(n.getNamespace(), "PointCloudOdometry");
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

bool PointCloudOdometry::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("PointCloudOdometry - LoadParameters");

  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_qx = 0.0, init_qy = 0.0, init_qz = 0.0, init_qw = 1.0;
  bool b_have_fiducial = true;

  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
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

  if (!pu::Get("icp/registration_method", params_.registration_method))
    return false;
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
  if (!pu::Get("icp/recompute_covariances", recompute_covariances_))
    return false;

  if (!pu::Get("b_verbose", b_verbose_))
    return false;
  if (!pu::Get("b_is_flat_ground_assumption", b_is_flat_ground_assumption_))
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

bool PointCloudOdometry::RegisterCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("PointCloudOdometry - RegisterCallbacks");
  ros::NodeHandle nl(n);
  query_pub_ = nl.advertise<PointCloudF>("odometry_query_points", 10, false);
  reference_pub_ =
      nl.advertise<PointCloudF>("odometry_reference_points", 10, false);
  incremental_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "odometry_incremental_estimate", 10, false);
  integrated_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>(
      "odometry_integrated_estimate", 10, false);
  return true;
}

bool PointCloudOdometry::SetupICP() {
  ROS_INFO("PointCloudOdometry - SetupICP");

  switch (getRegistrationMethodFromString(params_.registration_method)) {
  case RegistrationMethod::GICP: {
    ROS_INFO_STREAM("RegistrationMethod::GICP activated.");
    pcl::MultithreadedGeneralizedIterativeClosestPoint<PointF, PointF>::Ptr
        gicp = boost::make_shared<
            pcl::MultithreadedGeneralizedIterativeClosestPoint<PointF,
                                                               PointF>>();

    gicp->setTransformationEpsilon(params_.icp_tf_epsilon);
    gicp->setMaxCorrespondenceDistance(params_.icp_corr_dist);
    gicp->setMaximumIterations(params_.icp_iterations);
    gicp->setRANSACIterations(0);
    gicp->setNumThreads(params_.num_threads);
    gicp->enableTimingOutput(params_.enable_timing_output);
    gicp->RecomputeTargetCovariance(recompute_covariances_);
    gicp->RecomputeSourceCovariance(recompute_covariances_);
    gicp->setEuclideanFitnessEpsilon(0.005);
    ROS_INFO_STREAM("GICP");
    ROS_INFO_STREAM("getMaxCorrespondenceDistance: "
                    << gicp->getMaxCorrespondenceDistance());

    ROS_INFO_STREAM("getMaximumIterations: " << gicp->getMaximumIterations());
    ROS_INFO_STREAM(
        "getTransformationEpsilon: " << gicp->getTransformationEpsilon());
    ROS_INFO_STREAM(
        "getEuclideanFitnessEpsilon: " << gicp->getEuclideanFitnessEpsilon());

    ROS_INFO_STREAM("Ransac: " << gicp->getRANSACIterations());

    ROS_INFO_STREAM("getRANSACIterations: " << gicp->getRANSACIterations());
    ROS_INFO_STREAM(" ROTATION EPSILION: " << gicp->getRotationEpsilon());
    ROS_INFO_STREAM(
        "getCorrespondenceRandomness:" << gicp->getCorrespondenceRandomness());
    ROS_INFO_STREAM("getMaximumOptimizerIterations:"
                    << gicp->getMaximumOptimizerIterations());
    ROS_INFO_STREAM("getConvergeCriteria:" << gicp->getConvergeCriteria());
    ROS_INFO_STREAM(
        "RANSACOutlie: " << gicp->getRANSACOutlierRejectionThreshold());
    ROS_INFO_STREAM("CLASS NAME: " << gicp->getClassName());

    icp_ = gicp;
    break;
  }
  case RegistrationMethod::NDT: {
    ROS_INFO_STREAM("RegistrationMethod::NDT activated.");
    pclomp::NormalDistributionsTransform<PointF, PointF>::Ptr ndt_omp =
        boost::make_shared<
            pclomp::NormalDistributionsTransform<PointF, PointF>>();

    ndt_omp->setTransformationEpsilon(params_.icp_tf_epsilon);
    ndt_omp->setMaxCorrespondenceDistance(params_.icp_corr_dist);
    ndt_omp->setMaximumIterations(params_.icp_iterations);
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

void PointCloudOdometry::EnableOdometryIntegration() {
  b_use_odometry_integration_ = true;
  b_use_imu_integration_ = false; 
}

void PointCloudOdometry::EnableImuIntegration() {
  b_use_imu_integration_ = true;
  b_use_odometry_integration_ = false;
}

void PointCloudOdometry::DisableSensorIntegration() {
  b_use_imu_integration_ = false;
  b_use_odometry_integration_ = false;
}

bool PointCloudOdometry::SetLidar(const PointCloudF& points) {
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

bool PointCloudOdometry::UpdateEstimate() {
  if (!initialized_) {
    copyPointCloud(points_, *query_);
    initialized_ = true;
    return false;
  } else {
    copyPointCloud(*query_, *reference_);
    copyPointCloud(points_, *query_);
    return UpdateICP();
  }
}

bool PointCloudOdometry::UpdateICP() {
  query_trans_->clear();

  if (b_use_imu_integration_) {
    imu_prior_ << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    imu_prior_.block(0, 0, 3, 3) = imu_delta_;
    pcl::transformPointCloud(*query_, *query_trans_, imu_prior_);
  } else if (b_use_odometry_integration_) {
    Eigen::Matrix4f temp;
    pcl_ros::transformAsMatrix(odometry_delta_, temp);
    odometry_prior_ = temp.cast<double>();
    pcl::transformPointCloud(*query_, *query_trans_, odometry_prior_);
  } else {
    *query_trans_ = *query_;
  }

  icp_->setInputSource(query_trans_);
  icp_->setInputTarget(reference_);
  icp_->align(icpAlignedPointsOdometry_);
  Eigen::Matrix4d T;
  T = icp_->getFinalTransformation().cast<double>();

  if (b_use_imu_integration_) {
    T = T * imu_prior_;
  } else if (b_use_odometry_integration_) {
    T = T * odometry_prior_;
  } 

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
    incremental_estimate_.translation = gu::Vec3(T(0, 3), T(1, 3), 0);
    incremental_estimate_.rotation =
        gu::Rot3(cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1);
  } else {
    incremental_estimate_.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
    incremental_estimate_.rotation = gu::Rot3(T(0, 0),
                                              T(0, 1),
                                              T(0, 2),
                                              T(1, 0),
                                              T(1, 1),
                                              T(1, 2),
                                              T(2, 0),
                                              T(2, 1),
                                              T(2, 2));
  }

  if (!transform_thresholding_ ||
      (incremental_estimate_.translation.Norm() <= max_translation_ &&
       incremental_estimate_.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    integrated_estimate_ =
        gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  } else {
    ROS_WARN("%s: Discarding incremental transformation with norm (t: %lf, "
             "r: %lf)",
             name_.c_str(),
             incremental_estimate_.translation.Norm(),
             incremental_estimate_.rotation.ToEulerZYX().Norm());
  }

  // TODO: Improve the healthy check.
  is_healthy_ = true;

  return true;
}

void PointCloudOdometry::SetFlatGroundAssumptionValue(const bool& value) {
  ROS_INFO_STREAM(
      "PointCloudOdometry - SetFlatGroundAssumptionValue - Received: "
      << value);
  b_is_flat_ground_assumption_ = value;
  if (value)
    integrated_estimate_.rotation =
        gu::Rot3(0, 0, integrated_estimate_.rotation.Yaw());
}

const gu::Transform3& PointCloudOdometry::GetIncrementalEstimate() const {
  return incremental_estimate_;
}

const gu::Transform3& PointCloudOdometry::GetIntegratedEstimate() const {
  return integrated_estimate_;
}

bool PointCloudOdometry::GetLastPointCloud(PointCloudF::Ptr& out) const {
  if (!initialized_ || query_ == NULL) {
    ROS_WARN("%s: Not initialized.", name_.c_str());
    return false;
  }
  out = query_;
  return true;
}

void PointCloudOdometry::PublishAll() {
  PublishPose(incremental_estimate_, incremental_estimate_pub_);
  PublishPose(integrated_estimate_, integrated_estimate_pub_);
}

void PointCloudOdometry::PublishPose(const gu::Transform3& pose,
                                     const ros::Publisher& pub) {
  if (pub.getNumSubscribers() == 0)
    return;
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.pose = gr::ToRosPose(pose);
  ros_pose.header.frame_id = fixed_frame_id_;
  ros_pose.header.stamp = stamp_;
  pub.publish(ros_pose);
}

diagnostic_msgs::DiagnosticStatus PointCloudOdometry::GetDiagnostics() {
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
