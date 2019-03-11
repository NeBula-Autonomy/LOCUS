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
 * Author: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <laser_loop_closure/LaserLoopClosure.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>

#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <fstream>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

using gtsam::BetweenFactor;
using gtsam::ISAM2;
using gtsam::ISAM2Params;
using gtsam::NonlinearFactorGraph;
using gtsam::Pose3;
using gtsam::PriorFactor;
using gtsam::Rot3;
using gtsam::Values;
using gtsam::Vector3;
using gtsam::Vector6;

LaserLoopClosure::LaserLoopClosure()
    : key_(0), last_closure_key_(std::numeric_limits<int>::min()) {}

LaserLoopClosure::~LaserLoopClosure() {}

bool LaserLoopClosure::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "LaserLoopClosure");

  if (!filter_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }

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

bool LaserLoopClosure::LoadParameters(const ros::NodeHandle& n) {

  // Load frame ids.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_)) return false;
  if (!pu::Get("frame_id/base", base_frame_id_)) return false;

  // Should we turn loop closure checking on or off?
  if (!pu::Get("check_for_loop_closures", check_for_loop_closures_)) return false;

  // Optimizer selection
  if (!pu::Get("loop_closure_optimizer", loop_closure_optimizer_)) return false;
  

  // Load ISAM2 parameters.
  relinearize_skip_ = 1;
  relinearize_threshold_ = 0.01;
  if (!pu::Get("relinearize_skip", relinearize_skip_)) return false;
  if (!pu::Get("relinearize_threshold", relinearize_threshold_)) return false;
  if (!pu::Get("n_iterations_manual_loop_close", n_iterations_manual_loop_close_)) return false;

  // Load loop closing parameters.
  if (!pu::Get("translation_threshold", translation_threshold_)) return false;
  if (!pu::Get("proximity_threshold", proximity_threshold_)) return false;
  if (!pu::Get("max_tolerable_fitness", max_tolerable_fitness_)) return false;
  if (!pu::Get("skip_recent_poses", skip_recent_poses_)) return false;
  if (!pu::Get("poses_before_reclosing", poses_before_reclosing_)) return false;
  if (!pu::Get("manual_lc_rot_precision", manual_lc_rot_precision_)) return false;
  if (!pu::Get("manual_lc_trans_precision", manual_lc_trans_precision_)) return false;
  if (!pu::Get("laser_lc_rot_precision", laser_lc_rot_precision_)) return false;
  if (!pu::Get("laser_lc_trans_precision", laser_lc_trans_precision_)) return false;

  // Load ICP parameters.
  if (!pu::Get("icp/tf_epsilon", icp_tf_epsilon_)) return false;
  if (!pu::Get("icp/corr_dist", icp_corr_dist_)) return false;
  if (!pu::Get("icp/iterations", icp_iterations_)) return false;

  // Load initial position and orientation.
  double init_x = 0.0, init_y = 0.0, init_z = 0.0;
  double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
  if (!pu::Get("init/position/x", init_x)) return false;
  if (!pu::Get("init/position/y", init_y)) return false;
  if (!pu::Get("init/position/z", init_z)) return false;
  if (!pu::Get("init/orientation/roll", init_roll)) return false;
  if (!pu::Get("init/orientation/pitch", init_pitch)) return false;
  if (!pu::Get("init/orientation/yaw", init_yaw)) return false;

  // Load initial position and orientation noise.
  double sigma_x = 0.0, sigma_y = 0.0, sigma_z = 0.0;
  double sigma_roll = 0.0, sigma_pitch = 0.0, sigma_yaw = 0.0;
  if (!pu::Get("init/position_sigma/x", sigma_x)) return false;
  if (!pu::Get("init/position_sigma/y", sigma_y)) return false;
  if (!pu::Get("init/position_sigma/z", sigma_z)) return false;
  if (!pu::Get("init/orientation_sigma/roll", sigma_roll)) return false;
  if (!pu::Get("init/orientation_sigma/pitch", sigma_pitch)) return false;
  if (!pu::Get("init/orientation_sigma/yaw", sigma_yaw)) return false;

  // Create the ISAM2 solver.
  ISAM2Params parameters;
  parameters.relinearizeSkip = relinearize_skip_;
  parameters.relinearizeThreshold = relinearize_threshold_;
  isam_.reset(new ISAM2(parameters));

  // Set the initial position.
  Vector3 translation(init_x, init_y, init_z);
  Rot3 rotation(Rot3::RzRyRx(init_roll, init_pitch, init_yaw));
  Pose3 pose(rotation, translation);

  // Set the covariance on initial position.
  Vector6 noise;
  noise << sigma_x, sigma_y, sigma_z, sigma_roll, sigma_pitch, sigma_yaw;
  LaserLoopClosure::Diagonal::shared_ptr covariance(
      LaserLoopClosure::Diagonal::Sigmas(noise));

  // Initialize ISAM2.
  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakePriorFactor(pose, covariance));
  new_value.insert(key_, pose);

  isam_->update(new_factor, new_value);
  values_ = isam_->calculateEstimate();
  nfg_ = isam_->getFactorsUnsafe();
  key_++;

  // Set the initial odometry.
  odometry_ = Pose3::identity();

  return true;
}

// bool LaserLoopClosure::AddFactorService(laser_loop_closure::ManualLoopClosureRequest &request,
//                                         laser_loop_closure::ManualLoopClosureResponse &response) {
//   response.success = AddFactor(static_cast<unsigned int>(request.key_from),
//                                static_cast<unsigned int>(request.key_to));
//   if (response.success){
//     std::cout << "adding factor for loop closure succeeded" << std::endl;
//   }else{
//     std::cout << "adding factor for loop closure failed" << std::endl;
//   }

//   return true;
// }

bool LaserLoopClosure::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  odometry_edge_pub_ =
      nl.advertise<visualization_msgs::Marker>("odometry_edges", 10, false);
  loop_edge_pub_ =
      nl.advertise<visualization_msgs::Marker>("loop_edges", 10, false);
  graph_node_pub_ =
      nl.advertise<visualization_msgs::Marker>("graph_nodes", 10, false);
  graph_node_id_pub_ =
      nl.advertise<visualization_msgs::Marker>("graph_node_ids", 10, false);
  keyframe_node_pub_ =
      nl.advertise<visualization_msgs::Marker>("keyframe_nodes", 10, false);
  closure_area_pub_ =
      nl.advertise<visualization_msgs::Marker>("closure_area", 10, false);

  scan1_pub_ = nl.advertise<PointCloud>("loop_closure_scan1", 10, false);
  scan2_pub_ = nl.advertise<PointCloud>("loop_closure_scan2", 10, false);

  pose_graph_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph", 10, false);
  keyed_scan_pub_ =
      nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 10, false);
  loop_closure_notifier_pub_ =
      nl.advertise<std_msgs::Empty>("loop_closure", 10, false);

  // add_factor_srv_ = nl.advertiseService("add_factor", &LaserLoopClosure::AddFactorService, this);

  return true;
}

bool LaserLoopClosure::AddBetweenFactor(
    const gu::Transform3& delta, const LaserLoopClosure::Mat66& covariance,
    const ros::Time& stamp, unsigned int* key) {
  if (key == NULL) {
    ROS_ERROR("%s: Output key is null.", name_.c_str());
    return false;
  }

  // Append the new odometry.
  Pose3 new_odometry = ToGtsam(delta);

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenFactor(new_odometry, ToGtsam(covariance)));

  Pose3 last_pose = values_.at<Pose3>(key_-1);
  new_value.insert(key_, last_pose.compose(new_odometry));

  // Store this timestamp so that we can publish the pose graph later.
  keyed_stamps_.insert(std::pair<unsigned int, ros::Time>(key_, stamp));
  stamps_keyed_.insert(std::pair<double, unsigned int>(stamp.toSec(), key_));

  // Update ISAM2.
  try{
    isam_->update(new_factor, new_value); 
  } catch (...){
    // redirect cout to file
    std::ofstream nfgFile;
    std::string home_folder(getenv("HOME"));
    nfgFile.open(home_folder + "/Desktop/factor_graph.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(nfgFile.rdbuf());

    // save entire factor graph to file and debug if loop closure is correct
    gtsam::NonlinearFactorGraph nfg = isam_->getFactorsUnsafe();
    nfg.print();
    nfgFile.close();

    std::cout.rdbuf(coutbuf); //reset to standard output again

    ROS_ERROR("ISAM update error in AddBetweenFactors");
    throw;
  }
  
  // Update class variables
  values_ = isam_->calculateEstimate();

  nfg_ = isam_->getFactorsUnsafe();

  // Assign output and get ready to go again!
  *key = key_++;

  // We always add new poses, but only return true if the pose is far enough
  // away from the last one (keyframes). This lets the caller know when they
  // can add a laser scan.

  // Is the odometry translation large enough to add a new keyframe to the graph?
  odometry_ = odometry_.compose(new_odometry);
  if (odometry_.translation().norm() > translation_threshold_) {
    odometry_ = Pose3::identity();
    return true;
  }

  return false;
}

bool LaserLoopClosure::AddKeyScanPair(unsigned int key,
                                      const PointCloud::ConstPtr& scan) {
  if (keyed_scans_.count(key)) {
    ROS_ERROR("%s: Key %u already has a laser scan.", name_.c_str(), key);
    return false;
  }

  // The first key should be treated differently; we need to use the laser
  // scan's timestamp for pose zero.
  if (key == 0) {
    const ros::Time stamp = pcl_conversions::fromPCL(scan->header.stamp);
    keyed_stamps_.insert(std::pair<unsigned int, ros::Time>(key, stamp));
    stamps_keyed_.insert(std::pair<double, unsigned int>(stamp.toSec(), key));
  }

  ROS_INFO_STREAM("AddKeyScanPair " << key);

  // Add the key and scan.
  keyed_scans_.insert(std::pair<unsigned int, PointCloud::ConstPtr>(key, scan));

  // Publish the inserted laser scan.
  if (keyed_scan_pub_.getNumSubscribers() > 0) {
    pose_graph_msgs::KeyedScan keyed_scan;
    keyed_scan.key = key;

    pcl::toROSMsg(*scan, keyed_scan.scan);
    keyed_scan_pub_.publish(keyed_scan);
  }

  return true;
}

bool LaserLoopClosure::FindLoopClosures(
    unsigned int key, std::vector<unsigned int>* closure_keys) {
  // If loop closure checking is off, don't do this step. This will save some
  // computation time.
  if (!check_for_loop_closures_)
    return false;

  // Check arguments.
  if (closure_keys == NULL) {
    ROS_ERROR("%s: Output pointer is null.", name_.c_str());
    return false;
  }
  closure_keys->clear();

  // If a loop has already been closed recently, don't try to close a new one.
  if (std::fabs(key - last_closure_key_) < poses_before_reclosing_)
    return false;

  // Get pose and scan for the provided key.
  const gu::Transform3 pose1 = ToGu(values_.at<Pose3>(key));
  const PointCloud::ConstPtr scan1 = keyed_scans_[key];

  // Iterate through past poses and find those that lie close to the most
  // recently added one.
  bool closed_loop = false;
  for (const auto& keyed_pose : values_) {
    const unsigned int other_key = keyed_pose.key;

    // Don't self-check.
    if (other_key == key)
      continue;

    // Don't compare against poses that were recently collected.
    if (std::fabs(key - other_key) < skip_recent_poses_)
      continue;

    // Don't check for loop closures against poses that are not keyframes.
    if (!keyed_scans_.count(other_key))
      continue;

    const gu::Transform3 pose2 = ToGu(values_.at<Pose3>(other_key));
    const gu::Transform3 difference = gu::PoseDelta(pose1, pose2);
    if (difference.translation.Norm() < proximity_threshold_) {
      // Found a potential loop closure! Perform ICP between the two scans to
      // determine if there really is a loop to close.
      const PointCloud::ConstPtr scan2 = keyed_scans_[other_key];

      gu::Transform3 delta;
      LaserLoopClosure::Mat66 covariance;
      if (PerformICP(scan1, scan2, pose1, pose2, &delta, &covariance)) {
        // We found a loop closure. Add it to the pose graph.
        NonlinearFactorGraph new_factor;
        new_factor.add(BetweenFactor<Pose3>(key, other_key, ToGtsam(delta),
                                            ToGtsam(covariance)));
        isam_->update(new_factor, Values());
        closed_loop = true;
        last_closure_key_ = key;

        // Store for visualization and output.
        loop_edges_.push_back(std::make_pair(key, other_key));
        closure_keys->push_back(other_key);

        // Send an empty message notifying any subscribers that we found a loop
        // closure.
        loop_closure_notifier_pub_.publish(std_msgs::Empty());
      }
    }
  }
  values_ = isam_->calculateEstimate();

  nfg_ = isam_->getFactorsUnsafe();

  return closed_loop;
}

void LaserLoopClosure::GetMaximumLikelihoodPoints(PointCloud* points) {
  if (points == NULL) {
    ROS_ERROR("%s: Output point cloud container is null.", name_.c_str());
    return;
  }
  points->points.clear();

  // Iterate over poses in the graph, transforming their corresponding laser
  // scans into world frame and appending them to the output.
  for (const auto& keyed_pose : values_) {
    const unsigned int key = keyed_pose.key;

    // Check if this pose is a keyframe. If it's not, it won't have a scan
    // associated to it and we should continue.
    if (!keyed_scans_.count(key))
      continue;

    const gu::Transform3 pose = ToGu(values_.at<Pose3>(key));
    Eigen::Matrix4d b2w;
    b2w.block(0, 0, 3, 3) = pose.rotation.Eigen();
    b2w.block(0, 3, 3, 1) = pose.translation.Eigen();

    // Transform the body-frame scan into world frame.
    PointCloud scan_world;
    pcl::transformPointCloud(*keyed_scans_[key], scan_world, b2w);

    // Append the world-frame point cloud to the output.
    *points += scan_world;
  }
}

gu::Transform3 LaserLoopClosure::GetLastPose() const {
  if (key_ > 1) {
    return ToGu(values_.at<Pose3>(key_-1));
  } else {
    ROS_WARN("%s: The graph only contains its initial pose.", name_.c_str());
    return ToGu(values_.at<Pose3>(0));
  }
}

gu::Transform3 LaserLoopClosure::ToGu(const Pose3& pose) const {
  gu::Transform3 out;
  out.translation(0) = pose.translation().x();
  out.translation(1) = pose.translation().y();
  out.translation(2) = pose.translation().z();

  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j)
      out.rotation(i, j) = pose.rotation().matrix()(i, j);
  }

  return out;
}

Pose3 LaserLoopClosure::ToGtsam(const gu::Transform3& pose) const {
  Vector3 t;
  t(0) = pose.translation(0);
  t(1) = pose.translation(1);
  t(2) = pose.translation(2);

  Rot3 r(pose.rotation(0, 0), pose.rotation(0, 1), pose.rotation(0, 2),
         pose.rotation(1, 0), pose.rotation(1, 1), pose.rotation(1, 2),
         pose.rotation(2, 0), pose.rotation(2, 1), pose.rotation(2, 2));

  return Pose3(r, t);
}

LaserLoopClosure::Mat66 LaserLoopClosure::ToGu(
    const LaserLoopClosure::Gaussian::shared_ptr& covariance) const {
  gtsam::Matrix66 gtsam_covariance = covariance->covariance();

  LaserLoopClosure::Mat66 out;
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      out(i, j) = gtsam_covariance(i, j);

  return out;
}

LaserLoopClosure::Gaussian::shared_ptr LaserLoopClosure::ToGtsam(
    const LaserLoopClosure::Mat66& covariance) const {
  gtsam::Matrix66 gtsam_covariance;

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      gtsam_covariance(i, j) = covariance(i, j);

  return Gaussian::Covariance(gtsam_covariance);
}

PriorFactor<Pose3> LaserLoopClosure::MakePriorFactor(
    const Pose3& pose,
    const LaserLoopClosure::Diagonal::shared_ptr& covariance) {
  return PriorFactor<Pose3>(key_, pose, covariance);
}

BetweenFactor<Pose3> LaserLoopClosure::MakeBetweenFactor(
    const Pose3& delta,
    const LaserLoopClosure::Gaussian::shared_ptr& covariance) {
  odometry_edges_.push_back(std::make_pair(key_-1, key_));
  return BetweenFactor<Pose3>(key_-1, key_, delta, covariance);
}

bool LaserLoopClosure::PerformICP(const PointCloud::ConstPtr& scan1,
                                  const PointCloud::ConstPtr& scan2,
                                  const gu::Transform3& pose1,
                                  const gu::Transform3& pose2,
                                  gu::Transform3* delta,
                                  LaserLoopClosure::Mat66* covariance) {
  if (delta == NULL || covariance == NULL) {
    ROS_ERROR("%s: Output pointers are null.", name_.c_str());
    return false;
  }

  // Set up ICP.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(icp_tf_epsilon_);
  icp.setMaxCorrespondenceDistance(icp_corr_dist_);
  icp.setMaximumIterations(icp_iterations_);
  icp.setRANSACIterations(0);

  // Filter the two scans. They are stored in the pose graph as dense scans for
  // visualization.
  PointCloud::Ptr scan1_filtered(new PointCloud);
  PointCloud::Ptr scan2_filtered(new PointCloud);
  filter_.Filter(scan1, scan1_filtered);
  filter_.Filter(scan2, scan2_filtered);

  // Set source point cloud. Transform it to pose 2 frame to get a delta.
  const Eigen::Matrix<double, 3, 3> R1 = pose1.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> t1 = pose1.translation.Eigen();
  Eigen::Matrix4d body1_to_world;
  body1_to_world.block(0, 0, 3, 3) = R1;
  body1_to_world.block(0, 3, 3, 1) = t1;

  const Eigen::Matrix<double, 3, 3> R2 = pose2.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> t2 = pose2.translation.Eigen();
  Eigen::Matrix4d body2_to_world;
  body2_to_world.block(0, 0, 3, 3) = R2;
  body2_to_world.block(0, 3, 3, 1) = t2;

  PointCloud::Ptr source(new PointCloud);
  pcl::transformPointCloud(*scan1_filtered, *source, body1_to_world);
  icp.setInputSource(source);

  // Set target point cloud in its own frame.
  PointCloud::Ptr target(new PointCloud);
  pcl::transformPointCloud(*scan2_filtered, *target, body2_to_world);
  icp.setInputTarget(target);

  // Perform ICP.
  PointCloud unused_result;
  icp.align(unused_result);

  // Get resulting transform.
  const Eigen::Matrix4f T = icp.getFinalTransformation();
  gu::Transform3 delta_icp;
  delta_icp.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  delta_icp.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                T(1, 0), T(1, 1), T(1, 2),
                                T(2, 0), T(2, 1), T(2, 2));

  // Is the transform good?
  if (!icp.hasConverged())
    return false;

  if (icp.getFitnessScore() > max_tolerable_fitness_) {
    return false;
  }

  // Update the pose-to-pose odometry estimate using the output of ICP.
  const gu::Transform3 update =
      gu::PoseUpdate(gu::PoseInverse(pose1),
                     gu::PoseUpdate(gu::PoseInverse(delta_icp), pose1));

  *delta = gu::PoseUpdate(update, gu::PoseDelta(pose1, pose2));

  // TODO: Use real ICP covariance.
  covariance->Zeros();
  for (int i = 0; i < 3; ++i)
    (*covariance)(i, i) = laser_lc_rot_precision_; // 0.01
  for (int i = 3; i < 6; ++i)
    (*covariance)(i, i) = laser_lc_trans_precision_; // 0.04

  // If the loop closure was a success, publish the two scans.
  source->header.frame_id = fixed_frame_id_;
  target->header.frame_id = fixed_frame_id_;
  scan1_pub_.publish(*source);
  scan2_pub_.publish(*target);

  return true;
}

bool LaserLoopClosure::AddFactor(unsigned int key1, unsigned int key2) {
  // Thanks to Luca for providing the code
  ROS_INFO_STREAM("Adding factor between " << (int) key1 << " and " << (int) key2);

  // TODO - some check to see what the distance between the two poses are
  // Print that out for the operator to check - to see how large a change is being asked for

  // creating relative pose factor (also works for relative positions)
  gtsam::Pose3 measured = gtsam::Pose3(); // gtsam::Rot3(), gtsam::Point3();
  measured.print("Between pose is ");

  // create Information of measured
  gtsam::Vector6 precisions; // inverse of variances
  precisions.head<3>().setConstant(manual_lc_rot_precision_); // rotation precision
  precisions.tail<3>().setConstant(manual_lc_trans_precision_); // std: 1/1000 ~ 30 m 1/100 - 10 m 1/25 - 5m
  static const gtsam::SharedNoiseModel& loopClosureNoise =
  gtsam::noiseModel::Diagonal::Precisions(precisions);

  gtsam::Key id1 = key1; // more elegant way to “name” variables in GTSAM “Symbol” (x1,v1,b1)
  gtsam::Key id2 = key2;
  gtsam::BetweenFactor<gtsam::Pose3> factor(id1, id2, measured, loopClosureNoise);

  // TODO - remove debug messages 
  // Get the current offset and predict the error and cost
  gtsam::Pose3 p1 = values_.at<Pose3>(key1);
  gtsam::Pose3 p2 = values_.at<Pose3>(key2);
  gtsam::Point3 diff = p2.translation() - p1.translation();//p2.translation.distance(p1.translation);
  double err_d = diff.norm();
  double predicted_cost = (diff.x()*diff.x() + diff.y()*diff.y() + diff.z()*diff.z())*manual_lc_trans_precision_;
  ROS_INFO_STREAM("Vector between poses on loop closure is ");
  diff.print();
  ROS_INFO_STREAM("Distance between poses on loop closure is " << err_d); 
  ROS_INFO_STREAM("Predicted cost is " << predicted_cost); 


  gtsam::Values linPoint = isam_->getLinearizationPoint();
  double cost = factor.error(linPoint);
  ROS_INFO_STREAM("Cost of loop closure: " << cost); // 10^6 - 10^9 is ok (re-adjust covariances)  // cost = ( error )’ Omega ( error ), where the Omega = diag([0 0 0 1/25 1/25 1/25]). Error = [3 3 3] get an estimate for cost.
  // TODO get the positions of each of the poses and compute the distance between them - see what the error should be - maybe a bug there

  // add factor to factor graph
  NonlinearFactorGraph new_factor;
  new_factor.add(factor);


  // // save factor graph as graphviz dot file
  // // Render to PDF using "fdp Pose2SLAMExample.dot -Tpdf > graph.pdf"
  // std::ofstream os("Pose2SLAMExample.dot");
  // graph.saveGraph(os, result);

  // // Read File, create graph and initial estimate
  // // we are in build/examples, data is in examples/Data
  // NonlinearFactorGraph::shared_ptr graph;
  // Values::shared_ptr initial;
  // SharedDiagonal model = noiseModel::Diagonal::Sigmas((Vector(3) << 0.05, 0.05, 5.0 * M_PI / 180.0).finished());
  // string graph_file = findExampleDataFile("w100.graph");
  // boost::tie(graph, initial) = load2D(graph_file, model);
  // initial->print("Initial estimate:\n");

  // TODO - option to save or load the graph - check size of the existing graph?
  // Or just create an option to load an existing graph in the launch file?
  // TODO - Debug this for quicker testing - load a good graph that already has all we want.
  // bool bLoadGraph = false;
  // if (!bLoadGraph){
  //   // Save the graph 
  //   gtsam::SharedNoiseModel noise_model;
  //   noise_model = gtsam::noiseModel::Diagonal::Precisions::shared_ptr((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.04, 0.04, 0.04).finished());
  //   gtsam::save2D(isam_->getFactorsUnsafe(), values_, noise_model, "pre-loop_graph.graph");
  // }else{
  //   // Load the graph
  //   NonlinearFactorGraph::shared_ptr graph;
  //   Values::shared_ptr initial;
  //   gtsam::SharedNoiseModel noise_model = gtsam::noiseModel::Diagonal::Precisions::shared_ptr((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.04, 0.04, 0.04).finished());
  //   string graph_file = findExampleDataFile("pre-loop_graph.graph");
  //   boost::tie(graph, initial) = load2D(graph_file, noise_model);
  //   // initial->print("Initial estimate:\n");
  // }

  // optimize
  try {
    std::cout << "Optimizing manual loop closure, iteration" << std::endl;
    gtsam::ISAM2Result result_ISAM;
    // gtsam::NonlinearFactorGraph nfg;
    gtsam::Values initialEstimate;
    gtsam::Values result;

    // TODO - loop over optimizers?
    // TODO using strings or enum rather than ints
    // Switch based on optimizer input
    switch (loop_closure_optimizer_){
      case 0 : 
      {
        // Run isam 2 standard
        // TODO - test this!
        // isam_->update(new_factor, Values());
        // nfg = isam_->getFactorsUnsafe();
        // result = isam_->calculateEstimate();
        // Loop for n_iterations of the update 
        try{
        std::cout << "In ISAM2 update" << std::endl;
        for (int i = 0; i < n_iterations_manual_loop_close_; i++){
          std::cout << "Optimizing manual loop closure ISAM, iteration " << i << std::endl;
          if (i == 0){
            // Run first update with the added factors 
            result_ISAM = isam_->update(new_factor, Values());
          } else {
            // Run iterations of the update without adding new factors
            result_ISAM = isam_->update(NonlinearFactorGraph(), Values());
          }
          result_ISAM.print("iSAM2 update result:\t");

          linPoint = isam_->getLinearizationPoint();
          nfg_ = isam_->getFactorsUnsafe();
          cost = nfg_.error(linPoint);
          ROS_INFO_STREAM("iSAM2 Error at linearization point (after loop closure): " << cost); // 10^6 - 10^9 is ok (re-adjust covariances) 
        }
        result = isam_->calculateBestEstimate();
        }
        catch (...) {
          ROS_INFO_STREAM("Error with ISAM");
          throw;
        }
      }
        break;
      case 1 : 
      {
        // Levenber Marquardt Optimizer
        std::cout << "Running LM optimization" << std::endl;
        // nfg_ = isam_->getFactorsUnsafe();
        nfg_.add(factor);
        initialEstimate = isam_->calculateEstimate();
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosityLM("TRYDELTA");
        result = gtsam::LevenbergMarquardtOptimizer(nfg_, initialEstimate, params).optimize();
        // result.print("LM result is: ");
      }
        break;
      case 2 : 
      {
        // Dogleg Optimizer
        std::cout << "Running Dogleg optimization" << std::endl;
        // nfg_ = isam_->getFactorsUnsafe();
        nfg_.add(factor);
        initialEstimate = isam_->calculateEstimate();
        result = gtsam::DoglegOptimizer(nfg_, initialEstimate).optimize();
      }
        break;
      case 3 : 
      {
        // Gauss Newton Optimizer
        std::cout << "Running Gauss Newton optimization" << std::endl;
        // nfg_ = isam_->getFactorsUnsafe();
        nfg_.add(factor);

        // Optimise on the graph - set up parameters
        gtsam::GaussNewtonParams parameters;

        // Print per iteration
        parameters.setVerbosity("ERROR");

        // Optimize
        initialEstimate = isam_->calculateEstimate();
        result = gtsam::GaussNewtonOptimizer(nfg_, initialEstimate, parameters).optimize();
        
      }
        break;
      default : 
      {
        // Error
        ROS_INFO_STREAM("ERROR, wrong optimizer option");
        // TODO handle the error
      }
    }
    std::cout << "initial error = " << nfg_.error(initialEstimate) << std::endl;
    std::cout << "final error = " << nfg_.error(result) << std::endl;

    
    // ----------------------------------------------
    // Update results in ISAM2 - replace points?
    // Rest 
    // Create the ISAM2 solver.
    ISAM2Params parameters;
    parameters.relinearizeSkip = relinearize_skip_;
    parameters.relinearizeThreshold = relinearize_threshold_;
    isam_.reset(new ISAM2(parameters));
    
    // Update with the new graph
    isam_->update(nfg_,result); 
    




    



    // redirect cout to file
    std::ofstream nfgFile;
    std::string home_folder(getenv("HOME"));
    nfgFile.open(home_folder + "/Desktop/factor_graph.txt");
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(nfgFile.rdbuf());

    // save entire factor graph to file and debug if loop closure is correct
    // nfg = isam_->getFactorsUnsafe();
    nfg_.print();
    nfgFile.close();

    std::cout.rdbuf(coutbuf); //reset to standard output again
    
    // Store for visualization and output.
    loop_edges_.push_back(std::make_pair(id1, id2));

    // Send an empty message notifying any subscribers that we found a loop
    // closure.
    loop_closure_notifier_pub_.publish(std_msgs::Empty());

    // Update values
    // values_ = isam_->calculateEstimate();
    values_ = result;//

    // Todo test calculate best estimate vs calculate estimate
    // values =  isam_->calculateBestEstimate();

    // Get the current offset and predict the error and cost
    p1 = values_.at<Pose3>(key1);
    p2 = values_.at<Pose3>(key2);
    diff = p2.translation() - p1.translation();//p2.translation.distance(p1.translation);
    err_d = diff.norm();
    predicted_cost = (diff.x()*diff.x() + diff.y()*diff.y() + diff.z()*diff.z())*manual_lc_trans_precision_;
    ROS_INFO_STREAM("Vector between poses on loop closure is ");
    diff.print();
    ROS_INFO_STREAM("Distance between poses on loop closure is " << err_d); 
    ROS_INFO_STREAM("Predicted cost is " << predicted_cost); 
    linPoint = isam_->getLinearizationPoint();
    // nfg = isam_->getFactorsUnsafe();
    cost = nfg_.error(linPoint);
    ROS_INFO_STREAM("iSAM2 Error at linearization point (after loop closure): " << cost); // 10^6 - 10^9 is ok (re-adjust covariances) 

    // Publish
    PublishPoseGraph();

    return true; //result.getVariablesReeliminated() > 0;
  } catch (...) {
    ROS_ERROR("An error occurred while manually adding a factor to iSAM2.");
    throw;
  }
}

void LaserLoopClosure::PublishPoseGraph() {

  // Publish odometry edges.
  if (odometry_edge_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 0;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    m.color.a = 0.8;
    m.scale.x = 0.02;

    for (size_t ii = 0; ii < odometry_edges_.size(); ++ii) {
      unsigned int key1 = odometry_edges_[ii].first;
      unsigned int key2 = odometry_edges_[ii].second;

      gu::Vec3 p1 = ToGu(values_.at<Pose3>(key1)).translation;
      gu::Vec3 p2 = ToGu(values_.at<Pose3>(key2)).translation;

      m.points.push_back(gr::ToRosPoint(p1));
      m.points.push_back(gr::ToRosPoint(p2));
    }
    odometry_edge_pub_.publish(m);
  }

  // Publish loop closure edges.
  if (loop_edge_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 1;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 0.0;
    m.color.g = 0.2;
    m.color.b = 1.0;
    m.color.a = 0.8;
    m.scale.x = 0.02;

    for (size_t ii = 0; ii < loop_edges_.size(); ++ii) {
      unsigned int key1 = loop_edges_[ii].first;
      unsigned int key2 = loop_edges_[ii].second;

      gu::Vec3 p1 = ToGu(values_.at<Pose3>(key1)).translation;
      gu::Vec3 p2 = ToGu(values_.at<Pose3>(key2)).translation;

      m.points.push_back(gr::ToRosPoint(p1));
      m.points.push_back(gr::ToRosPoint(p2));
    }
    loop_edge_pub_.publish(m);
  }

  // Publish nodes in the pose graph.
  if (graph_node_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 2;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.color.r = 0.3;
    m.color.g = 0.0;
    m.color.b = 1.0;
    m.color.a = 0.8;
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;

    for (const auto& keyed_pose : values_) {
      gu::Vec3 p = ToGu(values_.at<Pose3>(keyed_pose.key)).translation;
      m.points.push_back(gr::ToRosPoint(p));
    }
    graph_node_pub_.publish(m);
  }

  // Publish node IDs in the pose graph.
  if (graph_node_id_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.2;
    m.color.a = 0.8;
    m.scale.z = 0.02; // Only Scale z is used - height of capital A in the text

    int id_base = 100;

    for (const auto& keyed_pose : values_) {
      gu::Transform3 p = ToGu(values_.at<Pose3>(keyed_pose.key));
      m.pose = gr::ToRosPose(p);
      // Display text for the node
      m.text = std::to_string(keyed_pose.key);
      m.id = id_base + keyed_pose.key;
      graph_node_id_pub_.publish(m);
    }
    
  }

  // Publish keyframe nodes in the pose graph.
  if (keyframe_node_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 3;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.3;
    m.color.a = 0.8;
    m.scale.x = 0.25;
    m.scale.y = 0.25;
    m.scale.z = 0.25;

    for (const auto& keyed_pose : values_) {
      if (keyed_scans_.count(keyed_pose.key)) {
        gu::Vec3 p = ToGu(values_.at<Pose3>(keyed_pose.key)).translation;
        m.points.push_back(gr::ToRosPoint(p));
      }
    }
    keyframe_node_pub_.publish(m);
  }

  // Draw a sphere around the current sensor frame to show the area in which we
  // are checking for loop closures.
  if (closure_area_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = base_frame_id_;
    m.ns = base_frame_id_;
    m.id = 4;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::SPHERE;
    m.color.r = 0.0;
    m.color.g = 0.4;
    m.color.b = 0.8;
    m.color.a = 0.4;
    m.scale.x = proximity_threshold_ * 2.0;
    m.scale.y = proximity_threshold_ * 2.0;
    m.scale.z = proximity_threshold_ * 2.0;
    m.pose = gr::ToRosPose(gu::Transform3::Identity());
    closure_area_pub_.publish(m);
  }

  // Construct and send the pose graph.
  if (pose_graph_pub_.getNumSubscribers() > 0) {
    pose_graph_msgs::PoseGraph g;
    g.header.frame_id = fixed_frame_id_;

    for (const auto& keyed_pose : values_) {
      gu::Transform3 t = ToGu(values_.at<Pose3>(keyed_pose.key));

      // Populate the message with the pose's data.
      pose_graph_msgs::PoseGraphNode node;
      node.key = keyed_pose.key;
      node.header.frame_id = fixed_frame_id_;
      node.pose = gr::ToRosPose(t);
      if (keyed_stamps_.count(keyed_pose.key)) {
        node.header.stamp = keyed_stamps_[keyed_pose.key];
      } else {
        ROS_WARN("%s: Couldn't find timestamp for key %lu", name_.c_str(),
                 keyed_pose.key);
      }
      g.nodes.push_back(node);
    }

    pose_graph_msgs::PoseGraphEdge edge;
    for (size_t ii = 0; ii < odometry_edges_.size(); ++ii) {
      edge.key_from = odometry_edges_[ii].first;
      edge.key_to = odometry_edges_[ii].second;
      g.edges.push_back(edge);
    }

    for (size_t ii = 0; ii < loop_edges_.size(); ++ii) {
      edge.key_from = loop_edges_[ii].first;
      edge.key_to = loop_edges_[ii].second;
      g.edges.push_back(edge);
    }

    // Publish.
    pose_graph_pub_.publish(g);
  }
}




gu::Transform3 LaserLoopClosure::GetPoseAtTime(const ros::Time& stamp) const {

  std::cout << "Get pose closest to input time: " << stamp.toSec() << std::endl;
  // Find closest timestamp
  // std::map<ros::Time, unsigned int>::iterator iterTime;
  
  // First key that is not less than timestamp (so the key just past the timestamp)
  auto iterTime = stamps_keyed_.lower_bound(stamp.toSec());

  std::cout << "Got iterator at lower_bound. Input: " << stamp.toSec() << ", found " << iterTime->first << std::endl;

  // TODO - interpolate - currently just take one
  double t2 = iterTime->first;
  double t1 = std::prev(iterTime,1)->first;

  std::cout << "Time 1 is: " << t1 << ", Time 2 is: " << t2 << std::endl;

  unsigned int key;

  if (t2-stamp.toSec() < stamp.toSec()-t1){
    // t2 is closer - use that key
    std::cout << "Selecting later time: " << t2 << std::endl;
    key = iterTime->second;
  }else {
    // t1 is closer - use that key
    std::cout << "Selecting earlier time: " << t1 << std::endl;
    key = std::prev(iterTime,1)->second;
    iterTime--;
  }
  std::cout << "Key is: " << key << std::endl;
  if (iterTime == std::prev(stamps_keyed_.begin())){
    ROS_WARN("Invalid time for graph (before start of graph range). Choosing next value");
    iterTime++;
    key = iterTime->second;
  } else if(iterTime==stamps_keyed_.end()){
    ROS_WARN("Invalid time for graph (past end of graph range). take latest pose");
    key = key_ -1;
  }

  // Get the pose at that key
  return ToGu(values_.at<Pose3>(key));
}
