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
#include <pcl/io/pcd_io.h>

#include <gtsam/slam/dataset.h>

#include <fstream>

#include <minizip/zip.h>
#include <minizip/unzip.h>

#include <time.h>

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
using gtsam::GraphAndValues;
using gtsam::Vector3;
using gtsam::Vector6;
using gtsam::ISAM2GaussNewtonParams;

LaserLoopClosure::LaserLoopClosure()
    : key_(0), last_closure_key_(std::numeric_limits<int>::min()) {
  initial_noise_.setZero();
}

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
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_)) return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_)) return false;
  if (!pu::Get("use_chordal_factor", use_chordal_factor_)) return false; 

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

  std::cout << "before isam reset" << std::endl; 
  #ifndef solver
  // Create the ISAM2 solver.
  ISAM2Params parameters;
  parameters.relinearizeSkip = relinearize_skip_;
  parameters.relinearizeThreshold = relinearize_threshold_;
  parameters.factorization = gtsam::ISAM2Params::QR; // QR
  // // Set wildfire threshold
  // ISAM2GaussNewtonParams gnparams(-1);
  // parameters.setOptimizationParams(gnparams);
  isam_.reset(new ISAM2(parameters));
  #endif
  #ifdef solver
  isam_.reset(new GenericSolver());
  isam_->print();
  #endif
  std::cout << "after isam reset" << std::endl; 

  // Set the initial position.
  Vector3 translation(init_x, init_y, init_z);
  Rot3 rotation(Rot3::RzRyRx(init_roll, init_pitch, init_yaw));
  Pose3 pose(rotation, translation);

  // Set the covariance on initial position.
  Vector6 noise;
  noise << sigma_roll, sigma_pitch, sigma_yaw, sigma_x, sigma_y, sigma_z;

  LaserLoopClosure::Diagonal::shared_ptr covariance(
      LaserLoopClosure::Diagonal::Sigmas(initial_noise_));

  // Initialize ISAM2.
  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakePriorFactor(pose, covariance));
  new_value.insert(key_, pose);

  isam_->update(new_factor, new_value);
  values_ = isam_->calculateEstimate();
  nfg_ = isam_->getFactorsUnsafe();
  std::cout << "!!!!! error at LoadParameters isam: " << nfg_.error(values_) << std::endl;
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

  std::cout << "!!!!! error at AddBetweenFactor isam: " << nfg_.error(values_) << std::endl;

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

bool LaserLoopClosure::AddBetweenChordalFactor(
    const gu::Transform3& delta, const LaserLoopClosure::Mat1212& covariance,
    const ros::Time& stamp, unsigned int* key) {
  if (key == NULL) {
    ROS_ERROR("%s: Output key is null.", name_.c_str());
    return false;
  }

  // Append the new odometry.
  Pose3 new_odometry = ToGtsam(delta);

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenChordalFactor(new_odometry, ToGtsam(covariance)));

  Pose3 last_pose = values_.at<Pose3>(key_-1);
  new_value.insert(key_, last_pose.compose(new_odometry));

  // Store this timestamp so that we can publish the pose graph later.
  keyed_stamps_.insert(std::pair<unsigned int, ros::Time>(key_, stamp));

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

  std::cout << "!!!!! error at AddBetweenFactor isam: " << nfg_.error(values_) << std::endl;

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
  }

  // ROS_INFO_STREAM("AddKeyScanPair " << key);

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

      if (!use_chordal_factor_) {
        gu::Transform3 delta; // (Using BetweenFactor)
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
      } else {

        gu::Transform3 delta; // (Using BetweenChordalFactor)
        LaserLoopClosure::Mat1212 covariance;
        if (PerformICP(scan1, scan2, pose1, pose2, &delta, &covariance)) {
          // We found a loop closure. Add it to the pose graph.
          NonlinearFactorGraph new_factor;
          new_factor.add(gtsam::BetweenChordalFactor<Pose3>(key, other_key, ToGtsam(delta),
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
  }
  values_ = isam_->calculateEstimate();

  nfg_ = isam_->getFactorsUnsafe();

  std::cout << "!!!!! error at FindLoopClosures isam: " << nfg_.error(values_) << std::endl;

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

LaserLoopClosure::Gaussian::shared_ptr LaserLoopClosure::ToGtsam(
    const LaserLoopClosure::Mat1212& covariance) const {
  gtsam::Vector12 gtsam_covariance; 
  // TODO CHECK
  for (int i = 0; i < 12; ++i) 
    gtsam_covariance(i) = covariance(i,i);
  return gtsam::noiseModel::Diagonal::Covariance(gtsam_covariance);
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

gtsam::BetweenChordalFactor<Pose3> LaserLoopClosure::MakeBetweenChordalFactor(
    const Pose3& delta, 
    const LaserLoopClosure::Gaussian::shared_ptr& covariance) {
  odometry_edges_.push_back(std::make_pair(key_-1, key_));
  return gtsam::BetweenChordalFactor<Pose3>(key_-1, key_, delta, covariance);
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
  // setVerbosityLevel(pcl::console::L_DEBUG);
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
  if (!icp.hasConverged()) {
    std::cout<<"No converged, score is: "<<icp.getFitnessScore() << std::endl;
    return false;
  }

  if (icp.getFitnessScore() > max_tolerable_fitness_) { 
      // std::cout<<"Trans: "<<delta_icp.translation<<std::endl;
      // std::cout<<"Rot: "<<delta_icp.rotation<<std::endl;
      // If the loop closure was a success, publish the two scans.
       std::cout<<"Converged, score is: "<<icp.getFitnessScore() << std::endl;
      // source->header.frame_id = fixed_frame_id_;
      // target->header.frame_id = fixed_frame_id_;
      // scan1_pub_.publish(*source);
      // scan2_pub_.publish(*target);
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
    (*covariance)(i, i) = laser_lc_rot_sigma_*laser_lc_rot_sigma_; 
  for (int i = 3; i < 6; ++i)
    (*covariance)(i, i) = laser_lc_trans_sigma_*laser_lc_trans_sigma_; 

  // If the loop closure was a success, publish the two scans.
  source->header.frame_id = fixed_frame_id_;
  target->header.frame_id = fixed_frame_id_;
  scan1_pub_.publish(*source);
  scan2_pub_.publish(*target);

  return true;
}

bool LaserLoopClosure::PerformICP(const PointCloud::ConstPtr& scan1,
                                  const PointCloud::ConstPtr& scan2,
                                  const gu::Transform3& pose1,
                                  const gu::Transform3& pose2,
                                  gu::Transform3* delta,
                                  LaserLoopClosure::Mat1212* covariance) {
  if (delta == NULL || covariance == NULL) {
    ROS_ERROR("%s: Output pointers are null.", name_.c_str());
    return false;
  }

  // Set up ICP.
  pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  // setVerbosityLevel(pcl::console::L_DEBUG);
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
  if (!icp.hasConverged()) {
    std::cout<<"No converged, score is: "<<icp.getFitnessScore() << std::endl;
    return false;
  }

  if (icp.getFitnessScore() > max_tolerable_fitness_) { 
      // std::cout<<"Trans: "<<delta_icp.translation<<std::endl;
      // std::cout<<"Rot: "<<delta_icp.rotation<<std::endl;
      // If the loop closure was a success, publish the two scans.
       std::cout<<"Converged, score is: "<<icp.getFitnessScore() << std::endl;
      // source->header.frame_id = fixed_frame_id_;
      // target->header.frame_id = fixed_frame_id_;
      // scan1_pub_.publish(*source);
      // scan2_pub_.publish(*target);
    return false;
  }

  // Update the pose-to-pose odometry estimate using the output of ICP.
  const gu::Transform3 update =
      gu::PoseUpdate(gu::PoseInverse(pose1),
                     gu::PoseUpdate(gu::PoseInverse(delta_icp), pose1));

  *delta = gu::PoseUpdate(update, gu::PoseDelta(pose1, pose2));

  // TODO: Use real ICP covariance.
  covariance->Zeros();
  for (int i = 0; i < 9; ++i)
    (*covariance)(i, i) = laser_lc_rot_sigma_*laser_lc_rot_sigma_;
  for (int i = 9; i < 12; ++i)
    (*covariance)(i, i) = laser_lc_trans_sigma_*laser_lc_trans_sigma_;

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

  gtsam::Key id1 = key1; // more elegant way to “name” variables in GTSAM “Symbol” (x1,v1,b1)
  gtsam::Key id2 = key2;

  NonlinearFactorGraph new_factor;

  std::cout << "isamgetlinearizationpoint-before" << std::endl; 
  gtsam::Values linPoint = isam_->getLinearizationPoint();
  std::cout << "isamgetlinearizationpoint-after" << std::endl; 

  nfg_ = isam_->getFactorsUnsafe();
  std::cout << "!!!!! error at AddFactor linpt: " << nfg_.error(linPoint) << std::endl;
  writeG2o(nfg_, linPoint, "/home/yunchang/Desktop/result_manual_loop_0.g2o");

  gtsam::Pose3 measured = gtsam::Pose3(gtsam::Rot3::Ypr(3.14, 0, 0), gtsam::Point3()); // gtsam::Rot3(), gtsam::Point3();
  measured.print("Between pose is ");

  double cost; // for debugging

  if (!use_chordal_factor_) {
    // Use BetweenFactor
    // creating relative pose factor (also works for relative positions)

    // create Information of measured
    gtsam::Vector6 precisions; // inverse of variances
    precisions.head<3>().setConstant(manual_lc_rot_precision_); // rotation precision
    precisions.tail<3>().setConstant(manual_lc_trans_precision_); // std: 1/1000 ~ 30 m 1/100 - 10 m 1/25 - 5m
    static const gtsam::SharedNoiseModel& loopClosureNoise =
    gtsam::noiseModel::Diagonal::Precisions(precisions);

    gtsam::BetweenFactor<gtsam::Pose3> factor(id1, id2, measured, loopClosureNoise);

    cost = factor.error(linPoint);
    ROS_INFO_STREAM("Cost of loop closure: " << cost); // 10^6 - 10^9 is ok (re-adjust covariances)  // cost = ( error )’ Omega ( error ), where the Omega = diag([0 0 0 1/25 1/25 1/25]). Error = [3 3 3] get an estimate for cost.
    // TODO get the positions of each of the poses and compute the distance between them - see what the error should be - maybe a bug there

    // add factor to factor graph
    new_factor.add(factor);

  } else {
    // Use BetweenChordalFactor  
    gtsam::Vector12 precisions; 
    precisions.head<9>().setConstant(manual_lc_rot_precision_); // rotation precision 
    precisions.tail<3>().setConstant(manual_lc_trans_precision_);
    static const gtsam::SharedNoiseModel& loopClosureNoise = 
    gtsam::noiseModel::Diagonal::Precisions(precisions);
 
    gtsam::BetweenChordalFactor<gtsam::Pose3> factor(id1, id2, measured, loopClosureNoise);

    cost = factor.error(linPoint);
    ROS_INFO_STREAM("Cost of loop closure: " << cost); // 10^6 - 10^9 is ok (re-adjust covariances)  // cost = ( error )’ Omega ( error ), where the Omega = diag([0 0 0 1/25 1/25 1/25]). Error = [3 3 3] get an estimate for cost.
    // TODO get the positions of each of the poses and compute the distance between them - see what the error should be - maybe a bug there
      
    // add factor to factor graph
    new_factor.add(factor);
  }


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
    gtsam::Values result, resultGN;

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
            isam_->update(new_factor, Values());
          } else {
            // Run iterations of the update without adding new factors
            isam_->update(NonlinearFactorGraph(), Values());
          }
          // result_ISAM.print("iSAM2 update result:\t");

          linPoint = isam_->calculateBestEstimate();
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
        // Levenberg Marquardt Optimizer
        std::cout << "Running LM optimization" << std::endl;
        isam_->update(new_factor, Values());
        initialEstimate = isam_->calculateEstimate();
        nfg_ = NonlinearFactorGraph(isam_->getFactorsUnsafe());
        // nfg_.print(""); // print whole factor graph
        std::cout << "number of factors after manual loop closure: " << nfg_.size() << std::endl; 
        std::cout << "number of poses after manual loop closure: " << initialEstimate.size() << std::endl; 
        // gtsam::Values chordalInitial = gtsam::InitializePose3::initialize(nfg_); // test
        // std::cout << "error at 1c: " << nfg_.error(chordalInitial) << std::endl;
        // writeG2o(nfg_, chordalInitial, "/home/yunchang/Desktop/result_manual_loop_1c.g2o");

        std::cout << "!!!!! error after isam2 update on manual loop closure: " << nfg_.error(initialEstimate) << std::endl;
        writeG2o(nfg_, initialEstimate, "/home/yunchang/Desktop/result_manual_loop_1.g2o");
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosityLM("TRYLAMBDA");
        result = gtsam::LevenbergMarquardtOptimizer(nfg_, linPoint, params).optimize();
        // result = gtsam::LevenbergMarquardtOptimizer(nfg_, initial, params).optimize();
        // result.print("LM result is: ");
        std::cout << "!!!!! error after LM on manual loop closure: " << nfg_.error(result) << std::endl;
        writeG2o(nfg_, result, "/home/yunchang/Desktop/result_manual_loop_2.g2o");

        //// Testing GN results 
        // gtsam::GaussNewtonParams paramsGN;
        // resultGN = gtsam::GaussNewtonOptimizer(nfg_, linPoint, paramsGN).optimize();
        // std::cout << "!!!!! error after GN on manual loop closure: " << nfg_.error(resultGN) << std::endl;
        // writeG2o(nfg_, resultGN, "/home/yunchang/Desktop/result_manual_loop_2gn.g2o");
      }
        break;
      case 2 : 
      {
        // Dogleg Optimizer
        std::cout << "Running Dogleg optimization" << std::endl;
        // nfg_ = isam_->getFactorsUnsafe();
        nfg_.add(new_factor);
        initialEstimate = isam_->calculateEstimate();
        result = gtsam::DoglegOptimizer(nfg_, initialEstimate).optimize();
      }
        break;
      case 3 : 
      {
        // Gauss Newton Optimizer
        std::cout << "Running Gauss Newton optimization" << std::endl;
        // nfg_ = isam_->getFactorsUnsafe();
        nfg_.add(new_factor);

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
    std::cout << "initial error = " << nfg_.error(linPoint) << std::endl;
    std::cout << "final error = " << nfg_.error(result) << std::endl;

    
    // ----------------------------------------------
    #ifndef solver
    // Create the ISAM2 solver.
    ISAM2Params parameters;
    parameters.relinearizeSkip = relinearize_skip_;
    parameters.relinearizeThreshold = relinearize_threshold_;
    // // Set wildfire threshold 
    // ISAM2GaussNewtonParams gnparams(-1);
    // parameters.setOptimizationParams(gnparams);
    isam_.reset(new ISAM2(parameters));
    #endif
    #ifdef solver
    isam_.reset(new GenericSolver());
    #endif
    // Update with the new graph
    isam_->update(nfg_,result); 
    gtsam::Values result_isam = isam_->calculateBestEstimate();
    std::cout << "!!!!! error after isam update from LM result: " << nfg_.error(result_isam) << std::endl;
    writeG2o(nfg_, result_isam, "/home/yunchang/Desktop/result_manual_loop_3.g2o");
    
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

std::string absPath(const std::string &relPath) {
  return boost::filesystem::canonical(boost::filesystem::path(relPath)).string();
}

bool writeFileToZip(zipFile &zip, const std::string &filename) {
  // this code is inspired by http://www.vilipetek.com/2013/11/22/zippingunzipping-files-in-c/
  static const unsigned int BUFSIZE = 2048;

  zip_fileinfo zi = {0};
  tm_zip& tmZip = zi.tmz_date;
  time_t rawtime;
  time(&rawtime);
  auto timeinfo = localtime(&rawtime);
  tmZip.tm_sec = timeinfo->tm_sec;
  tmZip.tm_min = timeinfo->tm_min;
  tmZip.tm_hour = timeinfo->tm_hour;
  tmZip.tm_mday = timeinfo->tm_mday;
  tmZip.tm_mon = timeinfo->tm_mon;
  tmZip.tm_year = timeinfo->tm_year;
  int err = zipOpenNewFileInZip(zip, filename.c_str(), &zi,
      NULL, 0, NULL, 0, NULL, Z_DEFLATED, Z_DEFAULT_COMPRESSION);

  if (err != ZIP_OK) {
    ROS_ERROR_STREAM("Failed to add entry \"" << filename << "\" to zip file.");
    return false;
  }
  char buf[BUFSIZE];
  unsigned long nRead = 0;

  std::ifstream is(filename);
  if (is.bad()) {
    ROS_ERROR_STREAM("Could not read file \"" << filename << "\" to be added to zip file.");
    return false;
  }
  while (err == ZIP_OK && is.good()) {
    is.read(buf, BUFSIZE);
    unsigned int nRead = (unsigned int)is.gcount();
    if (nRead)
      err = zipWriteInFileInZip(zip, buf, nRead);
    else
      break;
  }
  is.close();
  if (err != ZIP_OK) {
    ROS_ERROR_STREAM("Failed to write file \"" << filename << "\" to zip file.");
    return false;
  }
  return true;
}

bool LaserLoopClosure::Save(const std::string &zipFilename) const {
  const std::string path = "pose_graph";
  const boost::filesystem::path directory(path);
  boost::filesystem::create_directory(directory);

  writeG2o(isam_->getFactorsUnsafe(), values_, path + "/graph.g2o");
  ROS_INFO("Saved factor graph as a g2o file.");

  // keys.csv stores factor key, point cloud filename, and time stamp
  std::ofstream info_file(path + "/keys.csv");
  if (info_file.bad()) {
    ROS_ERROR("Failed to write info file.");
    return false;
  }

  auto zipFile = zipOpen64(zipFilename.c_str(), 0);
  writeFileToZip(zipFile, path + "/graph.g2o");
  int i = 0;
  for (const auto &entry : keyed_scans_) {
    info_file << entry.first << ",";
    // save point cloud as binary PCD file
    const std::string pcd_filename = path + "/pc_" + std::to_string(i) + ".pcd";
    pcl::io::savePCDFile(pcd_filename, *entry.second, true);
    writeFileToZip(zipFile, pcd_filename);

    ROS_INFO("Saved point cloud %d/%d.", i+1, (int) keyed_scans_.size());
    info_file << pcd_filename << ",";
    info_file << keyed_stamps_.at(entry.first).toNSec() << "\n";
    ++i;
  }
  info_file.close();
  writeFileToZip(zipFile, path + "/keys.csv");
  zipClose(zipFile, 0);
  boost::filesystem::remove_all(directory);
  ROS_INFO_STREAM("Successfully saved pose graph to " << absPath(zipFilename) << ".");
}

bool LaserLoopClosure::Load(const std::string &zipFilename) {
  const std::string absFilename = absPath(zipFilename);
  auto zipFile = unzOpen64(zipFilename.c_str());
  if (!zipFile) {
    ROS_ERROR_STREAM("Failed to open zip file " << absFilename);
    return false;
  }

  unz_global_info64 oGlobalInfo;
  int err = unzGetGlobalInfo64(zipFile, &oGlobalInfo);
  std::vector<std::string> files;  // files to be extracted

  bool foundGraph = false;
  bool foundKeys = false;

  std::string graphFilename, keysFilename;

  for (unsigned long i = 0; i < oGlobalInfo.number_entry && err == UNZ_OK; ++i) {
    char filename[256];
    unz_file_info64 oFileInfo;
    err = unzGetCurrentFileInfo64(zipFile, &oFileInfo, filename,
                                  sizeof(filename), NULL, 0, NULL, 0);
    if (err == UNZ_OK) {
      char nLast = filename[oFileInfo.size_filename-1];
      // this entry is a file, extract it later
      files.emplace_back(filename);
      if (files.back().find("graph.g2o") != std::string::npos) {
        foundGraph = true;
        graphFilename = files.back();
      } else if (files.back().find("keys.csv") != std::string::npos) {
        foundKeys = true;
        keysFilename = files.back();
      }
      err = unzGoToNextFile(zipFile);
    }
  }

  if (!foundGraph) {
    ROS_ERROR_STREAM("Could not find pose graph g2o-file in " << absFilename);
    return false;
  }
  if (!foundKeys) {
    ROS_ERROR_STREAM("Could not find keys.csv in " << absFilename);
    return false;
  }

  // extract files
  int i = 1;
  std::vector<boost::filesystem::path> folders;
  for (const auto &filename: files) {
    if (unzLocateFile(zipFile, filename.c_str(), 0) != UNZ_OK) {
			ROS_ERROR_STREAM("Could not locate file " << filename << " from " << absFilename);
      return false;
    }
    if (unzOpenCurrentFile(zipFile) != UNZ_OK) {
      ROS_ERROR_STREAM("Could not open file " << filename << " from " << absFilename);
      return false;
    }
    unz_file_info64 oFileInfo;
    if (unzGetCurrentFileInfo64(zipFile, &oFileInfo, 0, 0, 0, 0, 0, 0) != UNZ_OK)
    {
      ROS_ERROR_STREAM("Could not determine file size of entry " << filename << " in " << absFilename);
      return false;
    }

    boost::filesystem::path dir(filename);
    dir = dir.parent_path();
    if (boost::filesystem::create_directory(dir))
      folders.emplace_back(dir);

    auto size = (unsigned int) oFileInfo.uncompressed_size;
    char* buf = new char[size];
    size = unzReadCurrentFile(zipFile, buf, size);
    std::ofstream os(filename);
    if (os.bad()) {
      ROS_ERROR_STREAM("Could not create file " << filename << " for extraction.");
      return false;
    }
    if (size > 0) {
      os.write(buf, size);
      os.flush();
    } else {
      ROS_WARN_STREAM("Entry " << filename << " from " << absFilename << " is empty.");
    }
    os.close();
    delete [] buf;
    ROS_INFO_STREAM("Extracted file " << i << "/" << (int) files.size() << " -- " << filename);
    ++i;
  }
  unzClose(zipFile);

  // restore pose graph from g2o file
  const GraphAndValues gv = gtsam::load3D(graphFilename);
  nfg_ = *gv.first;
  values_ = *gv.second;

  #ifndef solver
  ISAM2Params parameters;
  parameters.relinearizeSkip = relinearize_skip_;
  parameters.relinearizeThreshold = relinearize_threshold_;
  isam_.reset(new ISAM2(parameters));
  #endif
  #ifdef solver
  isam_.reset(new GenericSolver());
  #endif

  const LaserLoopClosure::Diagonal::shared_ptr covariance(
      LaserLoopClosure::Diagonal::Sigmas(initial_noise_));
  const gtsam::Key key0 = *nfg_.keys().begin();
  nfg_.add(gtsam::PriorFactor<Pose3>(key0, values_.at<Pose3>(key0), covariance));
  isam_->update(nfg_, values_); 

  ROS_INFO_STREAM("Updated graph from " << graphFilename);

  // info_file stores factor key, point cloud filename, and time stamp
  std::ifstream info_file(keysFilename);
  if (info_file.bad()) {
    ROS_ERROR_STREAM("Failed to open " << keysFilename);
    return false;
  }

  std::string keyStr, pcd_filename, timeStr;
  while (info_file.good()) {
    std::getline(info_file, keyStr, ',');
    if (keyStr.empty())
      break;
    key_ = std::stoi(keyStr);
    std::getline(info_file, pcd_filename, ',');
    PointCloud::Ptr pc(new PointCloud);
    if (pcl::io::loadPCDFile(pcd_filename, *pc) == -1) {
      ROS_ERROR_STREAM("Failed to load point cloud " << pcd_filename << " from " << absFilename);
      return false;
    }
    ROS_INFO_STREAM("Loaded point cloud " << pcd_filename);
    keyed_scans_[key_] = pc;
    std::getline(info_file, timeStr);
    ros::Time t;
    t.fromNSec(std::stol(timeStr));
    keyed_stamps_[key_] = t;
  }

  ROS_INFO("Loaded all point clouds.");
  info_file.close();
  
  // remove all extracted folders
  for (const auto &folder: folders)
    boost::filesystem::remove_all(folder);

  ROS_INFO_STREAM("Successfully loaded pose graph from " << absPath(zipFilename) << ".");
  PublishPoseGraph();
  return true;
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

GenericSolver::GenericSolver(): 
  nfg_gs_(gtsam::NonlinearFactorGraph()),
  values_gs_(gtsam::Values()) {
  
  std::cout << "instantiated generic solver." << std::endl; 
}

void GenericSolver::update(gtsam::NonlinearFactorGraph nfg, gtsam::Values values) {
  nfg_gs_.add(nfg);
  values_gs_.insert(values);
  bool do_optimize = false; 

  // print number of loop closures
  std::cout << "number of loop closures so far: " << nfg_gs_.size() - values_gs_.size() << std::endl; 

  if (values.size() != 1) do_optimize = true; // for loop closure empty
  if (values.size() > 1) {
    ROS_WARN("Unexpected behavior: number of update poses greater than one.");
  }

  if (nfg.size() != 1) do_optimize = true; 
  if (nfg.size() > 1) {
    ROS_WARN("Unexpected behavior: number of update factors greater than one.");
  }

  if (nfg.size() == 0 && values.size() > 0) {
    ROS_ERROR("Unexpected behavior: added values but no factors.");
  }

  if (nfg.size() == 0 && values.size() == 0) do_optimize = false;

  if (nfg.size() == 1) {
    boost::shared_ptr<gtsam::BetweenFactor<Pose3> > pose3Between =
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<Pose3> >(nfg[0]);

    boost::shared_ptr<gtsam::BetweenChordalFactor<Pose3> > pose3BetweenChordal =
            boost::dynamic_pointer_cast<gtsam::BetweenChordalFactor<Pose3> >(nfg[0]);

    if (pose3Between || pose3BetweenChordal) {
      do_optimize = false;
    } else {
      ROS_WARN("Unexpected behavior: single not BetweenFactor factor added");
    }
  }

  if (do_optimize) {
    ROS_INFO(">>>>>>>>>>>> Run Optimizer <<<<<<<<<<<<");
    // optimize
    #if solver==LM
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("TRYLAMBDA");
    params.diagonalDamping = true; 
    values_gs_ = gtsam::LevenbergMarquardtOptimizer(nfg_gs_, values_gs_, params).optimize();
    #elif solver==SEsync
    // something
    #endif
  }
}
