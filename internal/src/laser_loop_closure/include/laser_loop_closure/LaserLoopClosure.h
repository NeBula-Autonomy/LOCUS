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

#ifndef LASER_LOOP_CLOSURE_H
#define LASER_LOOP_CLOSURE_H

#include <ros/ros.h>
#include <geometry_utils/Matrix3x3.h>
#include <geometry_utils/Transform3.h>
#include <point_cloud_filter/PointCloudFilter.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <pcl_ros/point_cloud.h>

#include <map>
#include <vector>

class LaserLoopClosure {
 public:
  LaserLoopClosure();
  ~LaserLoopClosure();

  bool Initialize(const ros::NodeHandle& n);

  // Typedef for 6x6 covariance matrices (x, y, z, roll, pitch, yaw).
  typedef geometry_utils::MatrixNxNBase<double, 6> Mat66;

  // Typedef for stored point clouds.
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  // Call this every time the robot's pose has been updated via ICP or some
  // other form of odometry. A between factor will always be added, but this
  // function will only return true when the new pose is significantly
  // different from the most recently added pose to enforce graph sparsity.
  // A return value of true lets the caller know when they should call
  // AddKeyScanPair().
  bool AddBetweenFactor(const geometry_utils::Transform3& delta,
                        const Mat66& covariance, const ros::Time& stamp,
                        unsigned int* key);

  // Upon successful addition of a new between factor, call this function to
  // associate a laser scan with the new pose.
  bool AddKeyScanPair(unsigned int key, const PointCloud::ConstPtr& scan);

  // After receiving an output key from 'AddBetweenFactor', call this to check
  // for loop closures with other poses in the pose graph.
  bool FindLoopClosures(unsigned int key,
                        std::vector<unsigned int>* closure_keys);

  // Build a 3D point cloud by concatenating all point clouds from poses along
  // the pose graph.
  void GetMaximumLikelihoodPoints(PointCloud* map);

  // Get the most recent pose in the pose graph.
  geometry_utils::Transform3 GetLastPose() const;

  // Publish pose graph for visualization.
  void PublishPoseGraph();

 private:

  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Pose conversion from/to GTSAM format.
  geometry_utils::Transform3 ToGu(const gtsam::Pose3& pose) const;
  gtsam::Pose3 ToGtsam(const geometry_utils::Transform3& pose) const;

  // Covariance conversion from/to GTSAM format.
  typedef gtsam::noiseModel::Gaussian Gaussian;
  typedef gtsam::noiseModel::Diagonal Diagonal;
  Mat66 ToGu(const Gaussian::shared_ptr& covariance) const;
  Gaussian::shared_ptr ToGtsam(const Mat66& covariance) const;

  // Create prior and between factors.
  gtsam::PriorFactor<gtsam::Pose3> MakePriorFactor(
      const gtsam::Pose3& pose, const Diagonal::shared_ptr& covariance);
  gtsam::BetweenFactor<gtsam::Pose3> MakeBetweenFactor(
      const gtsam::Pose3& pose, const Gaussian::shared_ptr& covariance);

  // Perform ICP between two laser scans.
  bool PerformICP(const PointCloud::ConstPtr& scan1,
                  const PointCloud::ConstPtr& scan2,
                  const geometry_utils::Transform3& pose1,
                  const geometry_utils::Transform3& pose2,
                  geometry_utils::Transform3* delta, Mat66* covariance);

  // Node name.
  std::string name_;

  // Keep a list of keyed laser scans and keyed timestamps.
  std::map<unsigned int, PointCloud::ConstPtr> keyed_scans_;
  std::map<unsigned int, ros::Time> keyed_stamps_;

  // Aggregate odometry until we can update the pose graph.
  gtsam::Pose3 odometry_;

  // Pose graph and ISAM2 parameters.
  bool check_for_loop_closures_;
  unsigned int key_;
  unsigned int last_closure_key_;
  unsigned int relinearize_interval_;
  unsigned int skip_recent_poses_;
  unsigned int poses_before_reclosing_;
  double translation_threshold_;
  double proximity_threshold_;
  double max_tolerable_fitness_;

  // ICP parameters.
  double icp_ransac_thresh_;
  double icp_tf_epsilon_;
  double icp_corr_dist_;
  unsigned int icp_iterations_;

  // ISAM2 optimizer object, and best guess pose values.
  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::Values values_;

  // Frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  // Visualization publishers.
  ros::Publisher odometry_edge_pub_;
  ros::Publisher loop_edge_pub_;
  ros::Publisher graph_node_pub_;
  ros::Publisher keyframe_node_pub_;
  ros::Publisher closure_area_pub_;
  ros::Publisher scan1_pub_;
  ros::Publisher scan2_pub_;

  // Pose graph publishers.
  ros::Publisher pose_graph_pub_;
  ros::Publisher keyed_scan_pub_;
  ros::Publisher loop_closure_notifier_pub_;

  typedef std::pair<unsigned int, unsigned int> Edge;
  std::vector<Edge> odometry_edges_;
  std::vector<Edge> loop_edges_;

  // For filtering laser scans prior to ICP.
  PointCloudFilter filter_;
};

#endif
