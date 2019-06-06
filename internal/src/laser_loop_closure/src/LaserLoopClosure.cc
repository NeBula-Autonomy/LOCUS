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
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

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

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

LaserLoopClosure::LaserLoopClosure()
    : key_(0), last_closure_key_(std::numeric_limits<int>::min()), tf_listener_(tf_buffer_) {
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

  // Should we save a backup pointcloud?
  if (!pu::Get("save_posegraph_backup", save_posegraph_backup_)) return false;

  // Should we save a backup pointcloud?
  if (!pu::Get("keys_between_each_posegraph_backup", keys_between_each_posegraph_backup_)) return false;

  // Optimizer selection
  if (!pu::Get("loop_closure_optimizer", loop_closure_optimizer_)) return false;
  

  // Load ISAM2 parameters.
  relinearize_skip_ = 1;
  relinearize_threshold_ = 0.01;
  if (!pu::Get("relinearize_skip", relinearize_skip_)) return false;
  if (!pu::Get("relinearize_threshold", relinearize_threshold_)) return false;
  if (!pu::Get("n_iterations_manual_loop_close", n_iterations_manual_loop_close_)) return false;

  // Load loop closing parameters.
  if (!pu::Get("translation_threshold_kf", translation_threshold_kf_))
    return false;
  if (!pu::Get("translation_threshold_nodes", translation_threshold_nodes_))
    return false;
  if (!pu::Get("proximity_threshold", proximity_threshold_)) return false;
  if (!pu::Get("max_tolerable_fitness", max_tolerable_fitness_)) return false;
  if (!pu::Get("skip_recent_poses", skip_recent_poses_)) return false;
  if (!pu::Get("poses_before_reclosing", poses_before_reclosing_)) return false;
  if (!pu::Get("manual_lc_rot_precision", manual_lc_rot_precision_)) return false;
  if (!pu::Get("manual_lc_trans_precision", manual_lc_trans_precision_)) return false;
  if (!pu::Get("laser_lc_rot_sigma", laser_lc_rot_sigma_)) return false;
  if (!pu::Get("laser_lc_trans_sigma", laser_lc_trans_sigma_)) return false;
  if (!pu::Get("artifact_rot_precision", artifact_rot_precision_)) return false; 
  if (!pu::Get("artifact_trans_precision", artifact_trans_precision_)) return false; 
  if (!pu::Get("use_chordal_factor", use_chordal_factor_))
    return false;
  if (!pu::Get("publish_interactive_markers", publish_interactive_markers_))
    return false;

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

  // Sanity check parameters
  if (!pu::Get("b_check_deltas", b_check_deltas_)) return false;
  if (!pu::Get("translational_sanity_check_lc", translational_sanity_check_lc_)) return false;
  if (!pu::Get("translational_sanity_check_odom", translational_sanity_check_odom_)) return false;

  std::cout << "before isam reset" << std::endl; 
  #ifndef SOLVER
  // Create the ISAM2 solver.
  ISAM2Params parameters;
  parameters.relinearizeSkip = relinearize_skip_;
  parameters.relinearizeThreshold = relinearize_threshold_;
  parameters.factorization = gtsam::ISAM2Params::QR; // QR
  // // Set wildfire threshold
  // ISAM2GaussNewtonParams gnparams(-1);
  // parameters.setOptimizationParams(gnparams);
  isam_.reset(new ISAM2(parameters));
  ROS_INFO("Using ISAM2 optimizer");
  #endif
  #ifdef SOLVER
  isam_.reset(new GenericSolver());
  isam_->print();
  ROS_INFO("Using generic solver (LM currently)");
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
  key_++;

  // Set the initial odometry.
  odometry_ = Pose3::identity();

  LAMP_recovery_ = false;
	//Initilize interactive marker server
  if (publish_interactive_markers_) {
    server.reset(new interactive_markers::InteractiveMarkerServer(
        "interactive_node", "", false));
  }

  return true;
}

bool LaserLoopClosure::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  odometry_edge_pub_ =
      nl.advertise<visualization_msgs::Marker>("odometry_edges", 10, false);
  loop_edge_pub_ =
      nl.advertise<visualization_msgs::Marker>("loop_edges", 10, false);
  artifact_edge_pub_ =
      nl.advertise<visualization_msgs::Marker>("artifact_edges", 10, false);
  graph_node_pub_ =
      nl.advertise<visualization_msgs::Marker>("graph_nodes", 10, false);
  graph_node_id_pub_ =
      nl.advertise<visualization_msgs::Marker>("graph_node_ids", 10, false);
  keyframe_node_pub_ =
      nl.advertise<visualization_msgs::Marker>("keyframe_nodes", 10, false);
  closure_area_pub_ =
      nl.advertise<visualization_msgs::Marker>("closure_area", 10, false);
  confirm_edge_pub_ =
  nl.advertise<visualization_msgs::Marker>("confirm_edge", 10, false);

  scan1_pub_ = nl.advertise<PointCloud>("loop_closure_scan1", 10, false);
  scan2_pub_ = nl.advertise<PointCloud>("loop_closure_scan2", 10, false);

  pose_graph_pub_ =
      nl.advertise<pose_graph_msgs::PoseGraph>("pose_graph", 10, false);
  keyed_scan_pub_ =
      nl.advertise<pose_graph_msgs::KeyedScan>("keyed_scans", 10, false);
  loop_closure_notifier_pub_ =
      nl.advertise<std_msgs::Empty>("loop_closure", 10, false);

  artifact_pub_ = nl.advertise<core_msgs::Artifact>("artifact", 10);
  marker_pub_ = nl.advertise<visualization_msgs::Marker>("artifact_markers", 10);
      
  return true;
}

bool LaserLoopClosure::AddFactorAtRestart(const gu::Transform3& delta, const LaserLoopClosure::Mat66& covariance){
  // Append the new odometry.
  Pose3 new_odometry = ToGtsam(delta);

  // Is the odometry translation large enough to add a new node to the graph
  odometry_ = odometry_.compose(new_odometry);
  //add a new factor
  Pose3 last_pose = values_.at<Pose3>(key_-1);

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenFactor(odometry_, ToGtsam(covariance)));
  new_value.insert(key_, last_pose.compose(odometry_));
  // TODO Compose covariances at the same time as odometry
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

  key_++;

  return true;
}

bool LaserLoopClosure::AddFactorAtLoad(const gu::Transform3& delta, const LaserLoopClosure::Mat66& covariance){
  // Append the new odometry.
  Pose3 new_odometry = ToGtsam(delta);

  // Is the odometry translation large enough to add a new node to the graph
  odometry_ = odometry_.compose(new_odometry);
  //add a new factor
  Pose3 first_pose = values_.at<Pose3>(0);

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenFactorAtLoad(odometry_, ToGtsam(covariance)));
  new_value.insert(key_, first_pose.compose(odometry_));
  // TODO Compose covariances at the same time as odometry
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

  key_++;

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

  // CHANGE TO CONFLICT THE BELOW STATEMENT
  // We always add new poses, but only return true if the pose is far enough
  // away from the last one (keyframes). This lets the caller know when they
  // can add a laser scan.

  // Is the odometry translation large enough to add a new node to the graph
  odometry_ = odometry_.compose(new_odometry);
  odometry_kf_ = odometry_kf_.compose(new_odometry);

  if (odometry_.translation().norm() < translation_threshold_nodes_) {
    // No new pose - translation is not enough to add a new node
    return false;
  }

  // Else - add a new factor

  NonlinearFactorGraph new_factor;
  Values new_value;
  new_factor.add(MakeBetweenFactor(odometry_, ToGtsam(covariance)));
  // TODO Compose covariances at the same time as odometry

  Pose3 last_pose = values_.at<Pose3>(key_-1);
  new_value.insert(key_, last_pose.compose(odometry_));

  // Compute cost before optimization
  NonlinearFactorGraph nfg_temp = isam_->getFactorsUnsafe();
  nfg_temp.add(new_factor);
  Values values_temp = isam_->getLinearizationPoint();
  values_temp.insert(key_, last_pose.compose(odometry_));
  double cost_old = nfg_temp.error(values_temp); // Assume values is up to date - no new values
  //ROS_INFO("Cost before optimization is: %f", cost_old);

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

  // Get updated cost
  double cost = nfg_.error(values_);

  //ROS_INFO("Cost after optimization is: %f", cost);

  // Do sanity check on result
  bool b_accept_update;
  if (b_check_deltas_ && values_backup_.exists(key_-1)){ // Only check if the values_backup has been stored (a loop closure has occured)
    ROS_INFO("Sanity checking output");
    b_accept_update = SanityCheckForLoopClosure(translational_sanity_check_odom_, cost_old, cost);
    // TODO - remove vizualization keys if it is rejected 

    if (!b_accept_update){
      ROS_WARN("Returning false for add between factor - have reset, waiting for next pose update");
      // Erase the current posegraph to make space for the backup
      LaserLoopClosure::ErasePosegraph();  
      // Run the load function to retrieve the posegraph
      LaserLoopClosure::Load("posegraph_backup.zip");
      return false;
    }
    ROS_INFO("Sanity check passed");
  }

  // Adding poses to stored hash maps
  // Store this timestamp so that we can publish the pose graph later.
  keyed_stamps_.insert(std::pair<unsigned int, ros::Time>(key_, stamp));
  stamps_keyed_.insert(std::pair<double, unsigned int>(stamp.toSec(), key_));

  // Assign output and get ready to go again!
  *key = key_++;

  // Reset odometry to identity
  odometry_ = Pose3::identity();

  // Return true to store a key frame
  if (odometry_kf_.translation().norm() > translation_threshold_kf_) {
    // True for a new key frame
    // Reset odometry to identity
    odometry_kf_ = Pose3::identity();
    return true;
  }

  return false;
}

//function to change keynumber for multiple robots
bool LaserLoopClosure::ChageKeyNumber(){
    key_ = 10000;
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

  // Assign output and get ready to go again!
  *key = key_++;

  // We always add new poses, but only return true if the pose is far enough
  // away from the last one (keyframes). This lets the caller know when they
  // can add a laser scan.

  // Is the odometry translation large enough to add a new keyframe to the graph?
  odometry_ = odometry_.compose(new_odometry);
  if (odometry_.translation().norm() > translation_threshold_kf_) {
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

  // Function to save the posegraph regularly
  if (key % keys_between_each_posegraph_backup_ == 0 && save_posegraph_backup_){
    LaserLoopClosure::Save("posegraph_backup.zip");
  } 

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

  // Update backups
  nfg_backup_ = isam_->getFactorsUnsafe();
  values_backup_ = isam_->getLinearizationPoint();

  // To track change in cost
  double cost;
  double cost_old;

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
            // Function to save the posegraph regularly
          if (save_posegraph_backup_){
            LaserLoopClosure::Save("posegraph_backup.zip");
          } 
          // We found a loop closure. Add it to the pose graph.
          NonlinearFactorGraph new_factor;
          new_factor.add(BetweenFactor<Pose3>(key, other_key, ToGtsam(delta),
                                              ToGtsam(covariance)));
          
          // Compute cost before optimization
          NonlinearFactorGraph nfg_temp = isam_->getFactorsUnsafe();
          nfg_temp.add(new_factor);
          cost_old = nfg_temp.error(values_); // Assume values is up to date - no new values


          // Optimization                                
          isam_->update(new_factor, Values());
          closed_loop = true;
          last_closure_key_ = key;

          // Get updated cost
          nfg_temp = isam_->getFactorsUnsafe();
          cost = nfg_temp.error(isam_->getLinearizationPoint());


          // Store for visualization and output.
          loop_edges_.push_back(std::make_pair(key, other_key));
          closure_keys->push_back(other_key);

          // Send an empty message notifying any subscribers that we found a loop
          // closure.
          loop_closure_notifier_pub_.publish(std_msgs::Empty());

          // break if a successful loop closure 
          // break;
        }
      } else {

        gu::Transform3 delta; // (Using BetweenChordalFactor)
        LaserLoopClosure::Mat1212 covariance;
        if (PerformICP(scan1, scan2, pose1, pose2, &delta, &covariance)) {
          if (save_posegraph_backup_){
            LaserLoopClosure::Save("posegraph_backup.zip");
          } 
          // We found a loop closure. Add it to the pose graph.
          NonlinearFactorGraph new_factor;
          new_factor.add(gtsam::BetweenChordalFactor<Pose3>(key, other_key, ToGtsam(delta),
                                              ToGtsam(covariance)));
          
          // Compute cost before optimization
          NonlinearFactorGraph nfg_temp = isam_->getFactorsUnsafe();
          nfg_temp.add(new_factor);
          cost_old = nfg_temp.error(values_); // Assume values is up to date - no new values

          // Optimization                                
          isam_->update(new_factor, Values());
          closed_loop = true;
          last_closure_key_ = key;

          // Get updated cost
          nfg_temp = isam_->getFactorsUnsafe();
          cost = nfg_temp.error(isam_->getLinearizationPoint());


          // Store for visualization and output.
          loop_edges_.push_back(std::make_pair(key, other_key));
          closure_keys->push_back(other_key);

          // Send an empty message notifying any subscribers that we found a loop
          // closure.
          loop_closure_notifier_pub_.publish(std_msgs::Empty());

          // break if a successful loop closure 
          // break;
        }
      }
      values_ = isam_->calculateEstimate();

      nfg_ = isam_->getFactorsUnsafe();

      // Check the change in pose to see if it exceeds criteria
      if (b_check_deltas_ && closed_loop){
        ROS_INFO("Sanity checking output");
        closed_loop = SanityCheckForLoopClosure(translational_sanity_check_lc_, cost_old, cost);
        // TODO - remove vizualization keys if it is rejected 
      
        if (!closed_loop){
          ROS_WARN("Returning false for bad loop closure - have reset, waiting for next pose update");
          // Erase the current posegraph to make space for the backup
          LaserLoopClosure::ErasePosegraph();
          // Run the load function to retrieve the posegraph  
          LaserLoopClosure::Load("posegraph_backup.zip");
          return false;
        }
    }
      // Update backups
      nfg_backup_ = nfg_;
      values_backup_ = values_;
    } // end of if statement 
  } // end of for loop
  
  return closed_loop;
}

bool LaserLoopClosure::SanityCheckForLoopClosure(double translational_sanity_check, double cost_old, double cost){
  // Checks loop closures to see if the translational threshold is within limits

  // if (!values_backup_.exists(key_-1)){
  //   ROS_INFO("")
  // }

  // Init poses
  gtsam::Pose3 old_pose;
  gtsam::Pose3 new_pose;

  if (key_ > 1){
    ROS_INFO("Key is more than 1, checking pose change");
    // Previous pose
    old_pose = values_backup_.at<Pose3>(key_-1);
    // New pose
    new_pose = values_.at<Pose3>(key_-1);
  }
  else{
    ROS_INFO("Key is less than or equal to 1, not checking pose change");
    // Second pose - return 
    return true;
  }

  // Translational change 
  double delta = old_pose.compose(new_pose.inverse()).translation().norm();
  
  ROS_INFO("Translational change with update is %f",delta);

  // TODO vary the sanity check values based on what kind of update it is
  // e.g. have the odom update to have a smaller sanity check 
  if (delta > translational_sanity_check || cost > cost_old){ // TODO - add threshold for error in the graph - if it increases after loop closure then reject
    if (delta > translational_sanity_check)
      ROS_WARN("Update delta exceeds threshold, rejecting");
    
    if (cost > cost_old)
      ROS_WARN("Cost increases, rejecting");

    // Updating 
    values_ = values_backup_;
    nfg_ = nfg_backup_;
    // Reset
    #ifndef SOLVER
    // Create the ISAM2 solver.
    ISAM2Params parameters;
    parameters.relinearizeSkip = relinearize_skip_;
    parameters.relinearizeThreshold = relinearize_threshold_;
    isam_.reset(new ISAM2(parameters));
    #endif
    #ifdef SOLVER
    isam_.reset(new GenericSolver());
    #endif
    ROS_INFO("Reset isam");
    isam_->update(nfg_, values_);
    ROS_INFO("updated isam to reset");

    // Save updated values
    values_ = isam_->calculateEstimate();
    nfg_ = isam_->getFactorsUnsafe();
    ROS_INFO("updated stored values");

    return false;
  }

  return true;

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

unsigned int LaserLoopClosure::GetKey() const {
  return key_;
}

gu::Transform3 LaserLoopClosure::GetLastPose() const {
  if (key_ > 1) {
    return ToGu(values_.at<Pose3>(key_-1));
  } else {
    ROS_WARN("%s: The graph only contains its initial pose.", name_.c_str());
    return ToGu(values_.at<Pose3>(0));
  }
}

gu::Transform3 LaserLoopClosure::GetInitialPose() const {
  if (key_ > 1) {
    return ToGu(values_.at<Pose3>(0));
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

BetweenFactor<Pose3> LaserLoopClosure::MakeBetweenFactorAtLoad(
    const Pose3& delta,
    const LaserLoopClosure::Gaussian::shared_ptr& covariance) {
  odometry_edges_.push_back(std::make_pair(0, key_));
  return BetweenFactor<Pose3>(0, key_, delta, covariance);
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

bool LaserLoopClosure::AddManualLoopClosure(gtsam::Key key1, gtsam::Key key2, 
                                            gtsam::Pose3 pose12){

  bool is_manual_loop_closure = true;
  return AddFactor(key1, key2, pose12, is_manual_loop_closure,
                   manual_lc_rot_precision_, manual_lc_trans_precision_); 
}

bool LaserLoopClosure::AddArtifact(gtsam::Key posekey, gtsam::Key artifact_key, 
                                   gtsam::Pose3 pose12, ArtifactInfo artifact) {

  // keep track of artifact info: add to hash if not added
  if (artifact_key2info_hash.find(artifact_key) == artifact_key2info_hash.end()) {
    ROS_INFO("New artifact detected with id %d",artifact.id);
    artifact_key2info_hash[artifact_key] = artifact;
  }
  // add to pose graph 
  bool is_manual_loop_closure = false;
  return AddFactor(posekey, artifact_key, pose12, is_manual_loop_closure,
                   artifact_rot_precision_, artifact_trans_precision_);
}

bool LaserLoopClosure::AddFactor(gtsam::Key key1, gtsam::Key key2, 
                                 gtsam::Pose3 pose12, 
                                 bool is_manual_loop_closure,
                                 double rot_precision, 
                                 double trans_precision) {
  // Thanks to Luca for providing the code
  ROS_INFO_STREAM("Adding factor between " << gtsam::DefaultKeyFormatter(key1) << " and " << gtsam::DefaultKeyFormatter(key2));

  gtsam::Values linPoint = isam_->getLinearizationPoint();
  nfg_ = isam_->getFactorsUnsafe();

  // Update backups
  nfg_backup_ = isam_->getFactorsUnsafe();
  values_backup_ = isam_->getLinearizationPoint();

  // Remove visualization of edge to be confirmed
  if (is_manual_loop_closure) {
    RemoveConfirmFactorVisualization();
    // check keys are already in factor graph 
    if (!linPoint.exists(key1) || !linPoint.exists(key2)) { 
      ROS_WARN("AddFactor: Trying to add manual loop closure involving at least one nonexisting key");
      return false;
    }
  }

  double cost_old;
  double cost; // for sanity checks

  NonlinearFactorGraph new_factor;
  gtsam::Values new_values;

  if (!is_manual_loop_closure && !linPoint.exists(key2)) {

    if(!linPoint.exists(key1)){
      ROS_WARN("AddFactor: Trying to add artifact factor, but key1 does not exist");
      return false;    
    }
    // We should add initial guess to values 
    new_values.insert(key2, linPoint.at<gtsam::Pose3>(key1).compose(pose12));

    ROS_INFO("Initial global position of artifact is: %f, %f, %f",
                  new_values.at<Pose3>(key2).translation().vector().x(),
                  new_values.at<Pose3>(key2).translation().vector().y(),
                  new_values.at<Pose3>(key2).translation().vector().z());
    

    // Set prior on rotation as a hack for 0 precision on rotation 
    gtsam::Vector6 prior_precisions; // inverse of variances
    prior_precisions.head<3>().setConstant(10.0); // rotation precision
    prior_precisions.tail<3>().setConstant(0.0); //
    static const gtsam::SharedNoiseModel& prior_noise =
    gtsam::noiseModel::Diagonal::Precisions(prior_precisions);

    new_factor.add(gtsam::PriorFactor<gtsam::Pose3>(key2, linPoint.at<gtsam::Pose3>(key1).compose(pose12), prior_noise));
  }

  linPoint.insert(new_values); // insert new values

  if (!use_chordal_factor_) {
    // Use BetweenFactor
    // creating relative pose factor (also works for relative positions)

    // create Information of measured
    gtsam::Vector6 precisions; // inverse of variances
    precisions.head<3>().setConstant(rot_precision); // rotation precision
    precisions.tail<3>().setConstant(trans_precision); // std: 1/1000 ~ 30 m 1/100 - 10 m 1/25 - 5m
    static const gtsam::SharedNoiseModel& noise =
    gtsam::noiseModel::Diagonal::Precisions(precisions);

    gtsam::BetweenFactor<gtsam::Pose3> factor(key1, key2, pose12, noise);

    if (is_manual_loop_closure){
      factor.print("manual loop closure factor \n");
      cost = factor.error(linPoint);
      ROS_INFO_STREAM("Cost of loop closure: " << cost); // 10^6 - 10^9 is ok (re-adjust covariances)  // cost = ( error )’ Omega ( error ), where the Omega = diag([0 0 0 1/25 1/25 1/25]). Error = [3 3 3] get an estimate for cost.
      // TODO get the positions of each of the poses and compute the distance between them - see what the error should be - maybe a bug there
    }
    else{
      factor.print("Artifact loop closure factor \n");
      cost = factor.error(linPoint);
      ROS_INFO_STREAM("Cost of artifact factor is: " << cost); 
    }  

    // add factor to factor graph
    new_factor.add(factor);

    // Store cost before optimization
    cost_old = new_factor.error(linPoint);


  } else {
    // Use BetweenChordalFactor  
    gtsam::Vector12 precisions; 
    precisions.head<9>().setConstant(rot_precision); // rotation precision 
    precisions.tail<3>().setConstant(trans_precision);
    static const gtsam::SharedNoiseModel& noise = 
    gtsam::noiseModel::Diagonal::Precisions(precisions);
 
    gtsam::BetweenChordalFactor<gtsam::Pose3> factor(key1, key2, pose12, noise);

    if (is_manual_loop_closure){
      factor.print("manual loop closure factor \n");
      cost = factor.error(linPoint);
      ROS_INFO_STREAM("Cost of loop closure: " << cost); // 10^6 - 10^9 is ok (re-adjust covariances)  // cost = ( error )’ Omega ( error ), where the Omega = diag([0 0 0 1/25 1/25 1/25]). Error = [3 3 3] get an estimate for cost.
      // TODO get the positions of each of the poses and compute the distance between them - see what the error should be - maybe a bug there
    }
    else{
      factor.print("Artifact loop closure factor \n");
      cost = factor.error(linPoint);
      ROS_INFO_STREAM("Cost of artifact factor is: " << cost); 
    }  

    // add factor to factor graph
    new_factor.add(factor);

    // Store cost before optimization
    cost_old = new_factor.error(linPoint);
  }

  // optimize
  try {
    if (is_manual_loop_closure){
      std::cout << "Optimizing manual loop closure, iteration" << std::endl;
    }else{
      std::cout << "Optimizing artifact factor addition" << std::endl;
    }
    gtsam::Values result;

    // Switch based on optimizer input
    switch (loop_closure_optimizer_){
      case 0 : // only do the above isam update 
      {
        // ISAM2
        isam_->update(new_factor, new_values);
        result = isam_->calculateEstimate();
        nfg_ = NonlinearFactorGraph(isam_->getFactorsUnsafe());
      }
        break;
      case 1 : 
      {
        // Levenberg Marquardt Optimizer
        nfg_.add(new_factor); // add new factor (new values already inserted above)
        std::cout << "Running LM optimization" << std::endl;
        gtsam::LevenbergMarquardtParams params;
        params.setVerbosityLM("SUMMARY");
        result = gtsam::LevenbergMarquardtOptimizer(nfg_, linPoint, params).optimize();
      }
        break;
      case 2 : 
      {
        // Dogleg Optimizer
        std::cout << "Running Dogleg optimization" << std::endl;
        nfg_.add(new_factor);
        result = gtsam::DoglegOptimizer(nfg_, linPoint).optimize();
      }
        break;
      case 3 : 
      { 
        // Gauss Newton Optimizer
        nfg_.add(new_factor); // add new factor (new values already inserted above)
        std::cout << "Running Gauss Newton optimization" << std::endl;
        gtsam::GaussNewtonParams params;
        params.setVerbosity("ERROR");
        result = gtsam::GaussNewtonOptimizer(nfg_, linPoint, params).optimize();
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
    #ifndef SOLVER
    // Create the ISAM2 solver.
    ISAM2Params parameters;
    parameters.relinearizeSkip = relinearize_skip_;
    parameters.relinearizeThreshold = relinearize_threshold_;
    isam_.reset(new ISAM2(parameters));
    #endif
    #ifdef SOLVER
    isam_.reset(new GenericSolver());
    #endif
    // Update with the new graph
    isam_->update(nfg_,result); 

    if (is_manual_loop_closure) {
      // Store for visualization and output.
      loop_edges_.push_back(std::make_pair(key1, key2));
      // Send an empty message notifying any subscribers that we found a loop
      // closure.
      loop_closure_notifier_pub_.publish(std_msgs::Empty());
    }
    else{
      // Placeholder visualization for artifacts
      artifact_edges_.push_back(std::make_pair(key1, key2));
    }

    // Update values
    values_ = result;//

    // INFO stream new cost
    linPoint = isam_->getLinearizationPoint();
    cost = nfg_.error(linPoint);
    ROS_INFO_STREAM("iSAM2 Error at linearization point (after loop closure): " << cost); // 10^6 - 10^9 is ok (re-adjust covariances) 

    // Check the change in pose to see if it exceeds criteria
    if (b_check_deltas_){
      ROS_INFO("Sanity checking output");
      bool check_result = SanityCheckForLoopClosure(translational_sanity_check_lc_, cost_old, cost);
    }

    // // Publish
    // PublishPoseGraph();

    return true; //result.getVariablesReeliminated() > 0;
  } catch (...) {
    ROS_ERROR("An error occurred while manually adding a factor to iSAM2.");
    throw;
  }
}

bool LaserLoopClosure::RemoveFactor(unsigned int key1, unsigned int key2) {
  ROS_INFO("Removing factor between %i and %i from the pose graph...", key1, key2);

  // Remove visualization of edge to be confirmed
  RemoveConfirmFactorVisualization();

  // Prevent removing odometry edges 
  if ((key1 == key2 - 1) || (key2 == key1 - 1)) {
    ROS_WARN("RemoveFactor: Removing edges from consecutive poses (odometry) is currently forbidden (disable if condition to allow)");
    return false; 
  }

  // 1. Get factor graph 
  NonlinearFactorGraph nfg = isam_->getFactorsUnsafe();
  // 2. Search for the two keys
  gtsam::FactorIndices factorsToRemove;
  for (size_t slot = 0; slot < nfg.size(); ++slot) {
    const gtsam::NonlinearFactor::shared_ptr& f = nfg[slot];
    if (f) {

      if (!use_chordal_factor_) {
        boost::shared_ptr<gtsam::BetweenFactor<Pose3> > pose3Between =
              boost::dynamic_pointer_cast<gtsam::BetweenFactor<Pose3> >(nfg[slot]);

        if (pose3Between) {
          if ((pose3Between->key1() == key1 && pose3Between->key2() == key2) ||
              (pose3Between->key1() == key2 && pose3Between->key2() == key1)) {
            factorsToRemove.push_back(slot);
            nfg[slot]->print("");
          }
        }

      } else { // using BetweenChordalFactor
        boost::shared_ptr<gtsam::BetweenChordalFactor<Pose3> > pose3Between =
              boost::dynamic_pointer_cast<gtsam::BetweenChordalFactor<Pose3> >(nfg[slot]);

        if (pose3Between) {
          if ((pose3Between->key1() == key1 && pose3Between->key2() == key2) ||
              (pose3Between->key1() == key2 && pose3Between->key2() == key1)) {
            factorsToRemove.push_back(slot);
            nfg[slot]->print("");
          }
        }
      }

    }

  }

  if (factorsToRemove.size() == 0) {
    ROS_WARN("RemoveFactor: Factor not found between given keys");
    return false; 
  }
  
  // 3. Remove factors and update
  std::cout << "Before remove update" << std::endl; 
  isam_->update(gtsam::NonlinearFactorGraph(), gtsam::Values(), factorsToRemove);

  // Send an empty message notifying any subscribers that we found a loop
  // closure.
  loop_closure_notifier_pub_.publish(std_msgs::Empty());

  // Update values
  values_ = isam_->calculateEstimate();

  // // Publish
  // PublishPoseGraph();

  return true; //result.getVariablesReeliminated() > 0;
}

bool LaserLoopClosure::VisualizeConfirmFactor(unsigned int key1, unsigned int key2) {
  ROS_INFO("Visualizing factor between %i and %i.", key1, key2);

  if (!values_.exists(key1) || !values_.exists(key2)) {
    ROS_WARN("Key %i or %i does not exist.", key1, key2);
    return false;
  }

  if ((key1 == key2 - 1) || (key2 == key1 - 1)) {
    ROS_WARN("Cannot add/remove factor between two consecutive keys.");
    return false;
  }

  visualization_msgs::Marker m;
  m.header.frame_id = fixed_frame_id_;
  m.ns = fixed_frame_id_;
  m.id = 0;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::LINE_LIST;
  m.color.r = 1.0;
  m.color.g = 1.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.scale.x = 0.05;
  const gu::Vec3 p1 = ToGu(values_.at<Pose3>(key1)).translation;
  const gu::Vec3 p2 = ToGu(values_.at<Pose3>(key2)).translation;

  m.points.push_back(gr::ToRosPoint(p1));
  m.points.push_back(gr::ToRosPoint(p2));
  confirm_edge_pub_.publish(m);
  
  m.header.frame_id = fixed_frame_id_;
  m.ns = fixed_frame_id_;
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::SPHERE;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.scale.x = 0.27;
  m.scale.y = 0.27;
  m.scale.z = 0.27;
  m.id = 1;
  m.pose.position = gr::ToRosPoint(p1);
  confirm_edge_pub_.publish(m);
  m.id = 2;
  m.pose.position = gr::ToRosPoint(p2);
  confirm_edge_pub_.publish(m);

  return true;
}

void LaserLoopClosure::RemoveConfirmFactorVisualization() {
  visualization_msgs::Marker m;
  m.header.frame_id = fixed_frame_id_;
  m.ns = fixed_frame_id_;
  m.id = 0;
  m.action = visualization_msgs::Marker::DELETEALL;
  confirm_edge_pub_.publish(m);
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

bool LaserLoopClosure::ErasePosegraph(){
  keyed_scans_.clear();
  keyed_stamps_.clear();
  stamps_keyed_.clear();

  loop_edges_.clear();
  odometry_ = Pose3::identity();
  odometry_kf_ = Pose3::identity();
  odometry_edges_.clear();

  key_ = 0;
  
	//Initilize interactive marker server
  if (publish_interactive_markers_) {
    server.reset(new interactive_markers::InteractiveMarkerServer(
        "interactive_node", "", false));
  }
} 

bool LaserLoopClosure::Save(const std::string &zipFilename) const {
  const std::string path = "pose_graph";
  const boost::filesystem::path directory(path);
  boost::filesystem::create_directory(directory);

  writeG2o(isam_->getFactorsUnsafe(), values_, path + "/graph.g2o");
  ROS_INFO("Saved factor graph as a g2o file.");

  // keys.csv stores factor key, point cloud filename, and time stamp
  std::ofstream keys_file(path + "/keys.csv");
  if (keys_file.bad()) {
    ROS_ERROR("Failed to write keys file.");
    return false;
  }

  auto zipFile = zipOpen64(zipFilename.c_str(), 0);
  writeFileToZip(zipFile, path + "/graph.g2o");
  int i = 0;
  for (const auto &entry : keyed_scans_) {
    keys_file << entry.first << ",";
    // save point cloud as binary PCD file
    const std::string pcd_filename = path + "/pc_" + std::to_string(i) + ".pcd";
    pcl::io::savePCDFile(pcd_filename, *entry.second, true);
    writeFileToZip(zipFile, pcd_filename);

    ROS_INFO("Saved point cloud %d/%d.", i+1, (int) keyed_scans_.size());
    keys_file << pcd_filename << ",";
    keys_file << keyed_stamps_.at(entry.first).toNSec() << "\n";
    ++i;
  }
  keys_file.close();
  writeFileToZip(zipFile, path + "/keys.csv");

  // save odometry edges
  std::ofstream odometry_edges_file(path + "/odometry_edges.csv");
  if (odometry_edges_file.bad()) {
    ROS_ERROR("Failed to write odometry_edges file.");
    return false;
  }
  for (const auto &entry : odometry_edges_) {
    odometry_edges_file << entry.first << ',' << entry.second << '\n';
  }
  odometry_edges_file.close();
  writeFileToZip(zipFile, path + "/odometry_edges.csv");

  // save loop edges
  std::ofstream loop_edges_file(path + "/loop_edges.csv");
  if (loop_edges_file.bad()) {
    ROS_ERROR("Failed to write loop_edges file.");
    return false;
  }
  for (const auto &entry : loop_edges_) {
    loop_edges_file << entry.first << ',' << entry.second << '\n';
  }
  loop_edges_file.close();
  writeFileToZip(zipFile, path + "/loop_edges.csv");

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

  std::string graphFilename{""}, keysFilename{""},
              odometryEdgesFilename{""}, loopEdgesFilename{""};

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
        graphFilename = files.back();
      } else if (files.back().find("keys.csv") != std::string::npos) {
        keysFilename = files.back();
      } else if (files.back().find("odometry_edges.csv") != std::string::npos) {
        odometryEdgesFilename = files.back();
      } else if (files.back().find("loop_edges.csv") != std::string::npos) {
        loopEdgesFilename = files.back();
      }
      err = unzGoToNextFile(zipFile);
    }
  }

  if (graphFilename.empty()) {
    ROS_ERROR_STREAM("Could not find pose graph g2o-file in " << absFilename);
    return false;
  }
  if (keysFilename.empty()) {
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

  #ifndef SOLVER
  ISAM2Params parameters;
  parameters.relinearizeSkip = relinearize_skip_;
  parameters.relinearizeThreshold = relinearize_threshold_;
  isam_.reset(new ISAM2(parameters));
  #endif
  #ifdef SOLVER
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
    //stamps_keyed_[t] = key_ ;
  }

  // Increment key to be ready for more scans
  key_++;

  ROS_INFO("Restored all point clouds.");
  info_file.close();

  if (!odometryEdgesFilename.empty()) {
    std::ifstream edge_file(odometryEdgesFilename);
    if (edge_file.bad()) {
      ROS_ERROR_STREAM("Failed to open " << odometryEdgesFilename);
      return false;
    }
    std::string edgeStr;
    while (edge_file.good()) {
      Edge edge;
      std::getline(edge_file, edgeStr, ',');
      if (edgeStr.empty())
        break;
      edge.first = static_cast<unsigned int>(std::stoi(edgeStr));
      std::getline(edge_file, edgeStr);
      edge.second = static_cast<unsigned int>(std::stoi(edgeStr));
      odometry_edges_.emplace_back(edge);
    }
    edge_file.close();
    ROS_INFO("Restored odometry edges.");
  }

  if (!loopEdgesFilename.empty()) {
    std::ifstream edge_file(loopEdgesFilename);
    if (edge_file.bad()) {
      ROS_ERROR_STREAM("Failed to open " << loopEdgesFilename);
      return false;
    }
    std::string edgeStr;
    while (edge_file.good()) {
      Edge edge;
      std::getline(edge_file, edgeStr, ',');
      if (edgeStr.empty())
        break;
      edge.first = static_cast<unsigned int>(std::stoi(edgeStr));
      std::getline(edge_file, edgeStr);
      edge.second = static_cast<unsigned int>(std::stoi(edgeStr));
      loop_edges_.emplace_back(edge);
    }
    edge_file.close();
    ROS_INFO("Restored loop closure edges.");
  }
  
  // remove all extracted folders
  for (const auto &folder: folders)
    boost::filesystem::remove_all(folder);

  LAMP_recovery_ = true;
  ROS_INFO_STREAM("Successfully loaded pose graph from " << absPath(zipFilename) << ".");
  PublishPoseGraph();
  return true;
}

//Interactive Marker Menu
void LaserLoopClosure::makeMenuMarker( gu::Transform3 position, const std::string id_number )
{
  interactive_markers::MenuHandler menu_handler;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = LaserLoopClosure::fixed_frame_id_;
  int_marker.scale = 1.0;
  int_marker.pose = gr::ToRosPose(position);
  int_marker.name = id_number;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 0.5;

  visualization_msgs::InteractiveMarkerControl control;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
  control.name = id_number;
  control.markers.push_back( marker );
  control.always_visible = true;
  int_marker.controls.push_back(control);

  menu_handler.insert(id_number);
  server->insert(int_marker);
  menu_handler.apply(*server, int_marker.name );
  //server->applyChanges();
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
      gtsam::Key key1 = odometry_edges_[ii].first;
      gtsam::Key key2 = odometry_edges_[ii].second;

      gu::Vec3 p1 = ToGu(values_.at<Pose3>(key1)).translation;
      gu::Vec3 p2 = ToGu(values_.at<Pose3>(key2)).translation;

      m.points.push_back(gr::ToRosPoint(p1));
      m.points.push_back(gr::ToRosPoint(p2));
    }
    odometry_edge_pub_.publish(m);
    // ros::spinOnce();
    // ros::Duration(0.005).sleep();
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
      gtsam::Key key1 = loop_edges_[ii].first;
      gtsam::Key key2 = loop_edges_[ii].second;

      gu::Vec3 p1 = ToGu(values_.at<Pose3>(key1)).translation;
      gu::Vec3 p2 = ToGu(values_.at<Pose3>(key2)).translation;

      m.points.push_back(gr::ToRosPoint(p1));
      m.points.push_back(gr::ToRosPoint(p2));
    }
    loop_edge_pub_.publish(m);
    // ros::spinOnce();
    // ros::Duration(0.005).sleep();
  }

  // Publish Artifact link messages
  if (artifact_edge_pub_.getNumSubscribers() > 0) {
    visualization_msgs::Marker m;
    m.header.frame_id = fixed_frame_id_;
    m.ns = fixed_frame_id_;
    m.id = 1;
    m.action = visualization_msgs::Marker::ADD;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.color.r = 0.2;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.6;
    m.scale.x = 0.02;

    for (size_t ii = 0; ii < artifact_edges_.size(); ++ii) {
      gtsam::Key key1 = artifact_edges_[ii].first;
      gtsam::Key key2 = artifact_edges_[ii].second;

      gu::Vec3 p1 = ToGu(values_.at<Pose3>(key1)).translation;
      gu::Vec3 p2 = ToGu(values_.at<Pose3>(key2)).translation;

      m.points.push_back(gr::ToRosPoint(p1));
      m.points.push_back(gr::ToRosPoint(p2));
    }
    artifact_edge_pub_.publish(m);
    // ros::spinOnce();
    // ros::Duration(0.005).sleep();
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
      std::string label = "l";
      if ((std::string(gtsam::Symbol(keyed_pose.key)).compare(0,1,label)) != 0){
        // If it is not a landmark keypose 
        gu::Vec3 p = ToGu(values_.at<Pose3>(keyed_pose.key)).translation;
        m.points.push_back(gr::ToRosPoint(p));
      }
    }
    graph_node_pub_.publish(m);
    // ros::spinOnce();
    // ros::Duration(0.005).sleep();
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
    int counter = 0;
    for (const auto& keyed_pose : values_) {
      gu::Transform3 p = ToGu(values_.at<Pose3>(keyed_pose.key));
      m.pose = gr::ToRosPose(p);
      // Display text for the node
      m.text = std::to_string(keyed_pose.key);
      m.id = id_base + keyed_pose.key;
      graph_node_id_pub_.publish(m);
      // if (counter % 500 == 0) {
        // throttle
        // ros::spinOnce();
        // ros::Duration(0.005).sleep();
      // }
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
    // ros::spinOnce();
    // ros::Duration(0.005).sleep();
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
    // ros::spinOnce();
    // ros::Duration(0.005).sleep();
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
  //Interactive Marker
  if (publish_interactive_markers_) {
    for (const auto& keyed_pose : values_) {
      if (keyed_pose.key % 1 == 0) {
        gu::Transform3 position = ToGu(values_.at<Pose3>(keyed_pose.key));
        const std::string id_number = std::to_string(keyed_pose.key);
        LaserLoopClosure::makeMenuMarker(position, id_number);
      }
    }
    if (server != NULL){
      server->applyChanges();
    }
  }
}

void LaserLoopClosure::PublishArtifacts(gtsam::Key artifact_key) {
  // For now, loop through artifact_key2label_hash
  // then publish. (might want to change this to an array later?)

  Eigen::Vector3d artifact_position;
  std::string artifact_label;

  // loop through values 
  for (auto it = artifact_key2info_hash.begin();
            it != artifact_key2info_hash.end(); ++it ) {
    
    if (artifact_key == '-1'){
      // Update all artifacts - loop through all - the default
      // Get position and label 
      artifact_position = GetArtifactPosition(it->first);
      artifact_label = it->second.label;

      // Increment update count
      it->second.num_updates++;
        
    }
    else{
      // Updating a single artifact - will return at the end of this first loop
      // Using the artifact key to publish that artifact
      ROS_INFO("Publishing only the new artifact");
      // Get position and label 
      artifact_position = GetArtifactPosition(artifact_key);
      artifact_label = artifact_key2info_hash[artifact_key].label;

      // Increment update count
      artifact_key2info_hash[artifact_key].num_updates++; 
    }

    // Create new artifact msg 
    core_msgs::Artifact new_msg;

    new_msg.point.point.x = artifact_position[0];
    new_msg.point.point.y = artifact_position[1];
    new_msg.point.point.z = artifact_position[2];
    new_msg.point.header.frame_id = fixed_frame_id_;
    // Transform to world frame from map frame
    new_msg.point = tf_buffer_.transform(
        new_msg.point, "world", new_msg.point.header.stamp, "world");
    // Transform at time of message
    std::cout << "Artifact position in world is: " << new_msg.point.point.x
              << ", " << new_msg.point.point.y << ", " << new_msg.point.point.z
              << std::endl;
    std::cout << "Frame ID is: " << new_msg.point.header.frame_id << std::endl;

    artifact_pub_.publish(new_msg);

    // Publish Marker with new position
    visualization_msgs::Marker marker;

    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "artifact";
    marker.id = it->first;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = new_msg.point.point;

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.35f;
    marker.scale.y = 0.35f;
    marker.scale.z = 0.35f;
    marker.color.a = 1.0f;

    if (artifact_label == "backpack")
    {
      std::cout << "backpack marker" << std::endl;
      marker.color.r = 1.0f;
      marker.color.g = 0.0f;
      marker.color.b = 0.0f;
      marker.type = visualization_msgs::Marker::CUBE;
    }
    if (artifact_label == "fire extinguisher")
    {
      std::cout << "fire extinguisher marker" << std::endl;
      marker.color.r = 1.0f;
      marker.color.g = 0.5f;
      marker.color.b = 0.75f;
      marker.type = visualization_msgs::Marker::SPHERE;
    }
    if (artifact_label == "drill")
    {
      std::cout << "drill marker" << std::endl;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.type = visualization_msgs::Marker::CYLINDER;
    }
    if (artifact_label == "survivor")
    {
      std::cout << "survivor marker" << std::endl;
      // return;
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 1.0f;
      marker.scale.x = 1.0f;
      marker.scale.y = 1.0f;
      marker.scale.z = 1.0f;
      marker.type = visualization_msgs::Marker::CYLINDER;
    }
    // marker.lifetime = ros::Duration();

    marker_pub_.publish(marker);

    if (artifact_key != '-1'){
      // Only a single artifact - exit the loop 
      return;
    }
  }
}

GenericSolver::GenericSolver(): 
  nfg_gs_(gtsam::NonlinearFactorGraph()),
  values_gs_(gtsam::Values()) {
  
  std::cout << "instantiated generic solver." << std::endl; 
}

void GenericSolver::update(gtsam::NonlinearFactorGraph nfg, 
                           gtsam::Values values, 
                           gtsam::FactorIndices factorsToRemove) {
  // remove factors
  for (size_t index : factorsToRemove) {
    nfg_gs_[index].reset();
  }

  // add new values and factors
  nfg_gs_.add(nfg);
  values_gs_.insert(values);
  bool do_optimize = true; 

  // print number of loop closures
  // std::cout << "number of loop closures so far: " << nfg_gs_.size() - values_gs_.size() << std::endl; 

  if (values.size() > 1) {ROS_WARN("Unexpected behavior: number of update poses greater than one.");}

  if (nfg.size() > 1) {ROS_WARN("Unexpected behavior: number of update factors greater than one.");}

  if (nfg.size() == 0 && values.size() > 0) {ROS_ERROR("Unexpected behavior: added values but no factors.");}

  // Do not optimize for just odometry additions 
  // odometry values would not have prefix 'l' unlike artifact values
  if (nfg.size() == 1 && values.size() == 1) {
    const gtsam::Symbol symb(values.keys()[0]); 
    if (symb.chr() != 'l') {do_optimize = false;}
  }

  // nothing added so no optimization 
  if (nfg.size() == 0 && values.size() == 0) {do_optimize = false;}

  if (factorsToRemove.size() > 0) 
    do_optimize = true;

  if (do_optimize) {
    ROS_INFO(">>>>>>>>>>>> Run Optimizer <<<<<<<<<<<<");
    // optimize
    #if SOLVER==1
    gtsam::LevenbergMarquardtParams params;
    params.setVerbosityLM("SUMMARY");
    std::cout << "Running LM" << std::endl; 
    params.diagonalDamping = true; 
    values_gs_ = gtsam::LevenbergMarquardtOptimizer(nfg_gs_, values_gs_, params).optimize();
    #elif SOLVER==2
    gtsam::GaussNewtonParams params;
    params.setVerbosity("ERROR");
    std::cout << "Running GN" << std::endl; 
    values_gs_ = gtsam::GaussNewtonOptimizer(nfg_gs_, values_gs_, params).optimize();
    #elif SOLVER==3
    // something
    #endif
  }
}

gtsam::Key LaserLoopClosure::GetKeyAtTime(const ros::Time& stamp) const {
  ROS_INFO("Get pose key closest to input time %f ", stamp.toSec());

  auto iterTime = stamps_keyed_.lower_bound(stamp.toSec()); // First key that is not less than timestamp 

  // std::cout << "Got iterator at lower_bound. Input: " << stamp.toSec() << ", found " << iterTime->first << std::endl;

  // TODO - interpolate - currently just take one
  double t2 = iterTime->first;
  double t1 = std::prev(iterTime,1)->first; 

  // std::cout << "Time 1 is: " << t1 << ", Time 2 is: " << t2 << std::endl;

  unsigned int key;

  if (t2-stamp.toSec() < stamp.toSec() - t1) { 
    // t2 is closer - use that key
    // std::cout << "Selecting later time: " << t2 << std::endl;
    key = iterTime->second;
  } else {
    // t1 is closer - use that key
    // std::cout << "Selecting earlier time: " << t1 << std::endl;
    key = std::prev(iterTime,1)->second;
    iterTime--;
  }
  // std::cout << "Key is: " << key << std::endl;
  if (iterTime == std::prev(stamps_keyed_.begin())){
    ROS_WARN("Invalid time for graph (before start of graph range). Choosing next value");
    iterTime = stamps_keyed_.begin();
    key = iterTime->second;
  } else if(iterTime == stamps_keyed_.end()){
    ROS_WARN("Invalid time for graph (past end of graph range). take latest pose");
    key = key_ -1;
  }

  return key; 
}

gu::Transform3 LaserLoopClosure::GetPoseAtKey(const gtsam::Key& key) const {
  // Get the pose at that key
  return ToGu(values_.at<Pose3>(key));
}

Eigen::Vector3d LaserLoopClosure::GetArtifactPosition(const gtsam::Key artifact_key) const {
  return values_.at<Pose3>(artifact_key).translation().vector();
}

/* TODO: 
-> AddBetweenFactor (and Chordal): rename to AddBetweenFactorOdometry 
-> AddBetweenChordalFactor: use ifdef to avoid copy paste 
-> move conversion functions to Utils.h
-> ideally laserLoopClosure should be split into LSLAMFrontEnd (icp), LSLAMBacknd (gtsam + posegraph)
-> AddFactor should be AddFactorManual (this will need a lot of cleaning after STIX) 
-> move all visualization functions to a Visualizer.h 
*/ 
