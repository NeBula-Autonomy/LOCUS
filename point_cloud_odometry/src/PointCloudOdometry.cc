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

  external_attitude_has_been_received_ = false;
  number_of_calls_ = 0;

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

  if (!b_have_fiducial) {
    ROS_WARN("Can't find fiducials, using origin");
  } else {
    ROS_INFO_STREAM("Have loaded fiducial pose, using:\n" << init);
  }
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

  if (!pu::Get("external_attitude/use_external_attitude", use_external_attitude_)) 
    return false;

  if (!pu::Get("frame_id/base", base_frame_id_))
    return false;
  if (!pu::Get("frame_id/imu", imu_frame_id_)) return false;

  // Load the tf calibration
  LoadCalibrationFromTfTree();

  return true;
}

bool PointCloudOdometry::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  query_pub_ = nl.advertise<PointCloud>("odometry_query_points", 10, false);
  reference_pub_ = nl.advertise<PointCloud>("odometry_reference_points", 10, false);
  incremental_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>("odometry_incremental_estimate", 10, false);
  integrated_estimate_pub_ = nl.advertise<geometry_msgs::PoseStamped>("odometry_integrated_estimate", 10, false);
  timestamp_difference_pub_ = nl.advertise<std_msgs::Float64>("external_attitude_lidar_ts_diff", 1, false); 

  return true;
}

void PointCloudOdometry::SetExternalAttitude(const geometry_msgs::Quaternion_<std::allocator<void>>& quaternion, const ros::Time& timestamp){  

  Eigen::Quaterniond q = Eigen::Quaterniond(double(quaternion.w), double(quaternion.x), double(quaternion.y), double(quaternion.z));

  // Transform incoming quaternion from imu to base_link frames
  q = I_T_B_q_*q*I_T_B_q_.inverse();

  // Buffering external attitude in FIFO Structure - an optimal deque size increases speed and decreases computational load when a search in the deque is performed (syncing stage)
  if (external_attitude_deque_.size()==max_external_attitude_deque_size_){ 
    external_attitude_deque_.pop_front();
    external_attitude_deque_.push_back(external_attitude{q, timestamp});
  }
  else{
    external_attitude_deque_.push_back(external_attitude{q, timestamp});
  }

  if (external_attitude_has_been_received_==false){
    external_attitude_first_ = q; // First time receiving the external attitude  
    std::cout << "Receiving external attitude for the first time ---> external_first_attitude_ now exists!";
    external_attitude_has_been_received_ = true;
  }
}

bool PointCloudOdometry::UpdateEstimate(const PointCloud& points) {

  // As soon as UpdateEstimate is called, a copy of the deque of interest is taken in order to not pick the wrong i-th element when external producer threads are pushing element into the deque
  std::deque<external_attitude> external_attitude_deque_copy = external_attitude_deque_; 
  
  // Store input point cloud's time stamp for publishing.
  stamp_.fromNSec(points.header.stamp * 1e3);

  // If this is the first point cloud, store it and wait for anotherinverse.
    if (!initialized_) {
    if (use_external_attitude_==true){
      copyPointCloud(points, *query_); 
      if (external_attitude_has_been_received_ == true){
        external_attitude_previous_ = external_attitude_first_; 
        initialized_ = true;
        return false;
      }
      else{
        /* ------------- Deactivate external attitude usage when external provider crashes at start ------------- 
        
        DOCUMENTATION:  - What about if external attitude provider crashes at start and we never receive a first attitude? 
                          Odometry would be stuck and no poses would be created.

                        - We could handle this maybe creating a counter that keeps track of how many times 
                          the UpdateEstimate() method has been called with a new incoming PointCloudand if after 
                          25 calls has still not been initialized, we deactivate the use_external_attitude_ usage 
                          and  proceed with pure ICP Lidar. 
        */
        number_of_calls_ = number_of_calls_ + 1; 
        if (number_of_calls_==25){
          ROS_WARN("UpdateEstimate has been called 25 times, but no external attitude has been received yet.");
          ROS_WARN("Deactivating external attitude usage and relying on pure ICP Lidar now.");
          use_external_attitude_ = false; 
        }

        return false;
      }      
    }
    else{
      copyPointCloud(points, *query_);
      initialized_ = true;
      return false;      
    }
  }

  /* ------------- Deactivate external attitude usage when external provider crashes at any time ------------- 

  DOCUMENTATION:  - We're assuming that the external attitude is provided at a constant velocity
                  
                  - If there’s a drop in the received message rate, we would see, for a given constant update rate, a deque_size of 99 rather than the expected 100 
                    (numerical values setted for min_external_attitude_deque_size_ and max_external_attitude_deque_size_ respectively).
                  
                  - Lowering the min_external_attitude_deque_size would be a more flexible condition but would have some effects on the generated odometry. 

                  - Alternative approaches to this could be also elaborated (e.g. timeout on the subscription level). 
  */
  if(external_attitude_deque_copy.size()==min_external_attitude_deque_size_){
    use_external_attitude_ = false; 
    ROS_WARN("External attitude data provider crashed - Relying now on pure ICP"); 
  }
  
  if(use_external_attitude_==true){

    /* ------------- SYNCING STAGE -------------  
    Choose the closest external attitude signal in respect to the timestamp of the current received LIDAR scan
    */ 

    external_attitude_current_ = external_attitude_deque_copy[0].internal_external_attitude_; 
    double min_ts_diff = 1000;   
    for (int i=0; i<external_attitude_deque_copy.size(); ++i) {
          double cur_ts_diff = (external_attitude_deque_copy[i].internal_external_attitude_timestamp_ - stamp_).toSec();
          if (cur_ts_diff<0 && fabs(cur_ts_diff)<fabs(min_ts_diff)){
              external_attitude_current_ = external_attitude_deque_copy[i].internal_external_attitude_; 
              min_ts_diff = cur_ts_diff; 
          }
    }

    // Warn user if the selected external attitude comes from the future or too far from the past 
    if (min_ts_diff>0){
      ROS_WARN("WARNING: External attitude comes from the future");
    }
    if (min_ts_diff<0 && fabs(min_ts_diff)>fabs(0.1)){
      ROS_WARN("WARNING: External attitude comes from the past, but it's too old");
    }
    // TODO: At this point, if needed, we can deactivate the external attitude usage 

    std_msgs::Float64 external_attitude_lidar_ts_diff; 
    external_attitude_lidar_ts_diff.data = min_ts_diff; 
    PublishTimestampDifference(external_attitude_lidar_ts_diff, timestamp_difference_pub_); 
    
    // Compute the change in attitude, make a copy of it and store it in the deque
    external_attitude_change_ = external_attitude_previous_.inverse()*external_attitude_current_;  
    Eigen::Quaterniond external_change_in_attitude_copy = external_attitude_change_;     
    external_attitude_change_deque_.push_back(external_change_in_attitude_copy);  

    // Move current query points (acquired last iteration) to reference points.
    copyPointCloud(*query_, *reference_);

    // Set the incoming point cloud as the query point cloud.
    copyPointCloud(points, *query_);

    // Update external attitude previous
    external_attitude_previous_ = external_attitude_current_; 

    // Update pose estimate via ICP.
    return UpdateICP();
  }

  else{
    // Move current query points (acquired last iteration) to reference points.
    copyPointCloud(*query_, *reference_);

    // Set the incoming point cloud as the query point cloud.
    copyPointCloud(points, *query_);

    // Update previous attitude
    external_attitude_previous_ = external_attitude_current_; 

    // Update pose estimate via ICP.
    return UpdateICP();
  }
  
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
  PointCloud unused_result;
  icp.align(unused_result);

  Eigen::Matrix4d T; 

  if (use_external_attitude_==true){
    // ------------------- //
    // Process the ICP
    T = icp.getFinalTransformation().cast<double>(); 

    // Get the yaw from the 
    tf::Matrix3x3 m_icp(T(0,0),T(0,1),T(0,2),T(1,0),T(1,1),T(1,2),T(2,0),T(2,1),T(2,2));
    double roll, pitch, yaw;
    m_icp.getRPY(roll, pitch, yaw);
    /* remove pitch and roll, just want yaw */
    Eigen::Matrix3d rot_yaw_mat;
    rot_yaw_mat << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

    // Yaw only 
    Eigen::Quaterniond q_yaw_icp(rot_yaw_mat);

    // ------------------- //
    // Process IMU attitude
    Eigen::Quaterniond external_attitude_local_copy = external_attitude_change_deque_.front();  

    tf::Quaternion q0(external_attitude_local_copy.x(),
                        external_attitude_local_copy.y(),
                        external_attitude_local_copy.z(),
                        external_attitude_local_copy.w());
    tf::Matrix3x3 m(q0);
    m.getRPY(roll, pitch, yaw);
    /* remove pitch and roll, just want yaw */
    rot_yaw_mat = Eigen::Matrix3d();
    rot_yaw_mat << cos(yaw), -sin(yaw), 0, sin(yaw), cos(yaw), 0, 0, 0, 1;

    // Yaw only 
    Eigen::Quaterniond q_yaw(rot_yaw_mat);

    // Take away the yaw
    Eigen::Quaterniond q_imu_roll_pitch = q_yaw.inverse()*external_attitude_local_copy;

    // ------------------- //
    // Compute attitude - yaw from ICP, roll and pitch from IMU
    Eigen::Quaterniond q_out = q_yaw_icp*q_imu_roll_pitch;

    external_attitude_change_deque_.pop_front();
    T.block(0,0,3,3) = q_out.toRotationMatrix();       
    // std::cout << "external attitude usage ON" << std::endl;  
  }
  else{
    T = icp.getFinalTransformation().cast<double>();
    // std::cout << "external attitude usage OFF" << std::endl;  
  }

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
                                             
void PointCloudOdometry::PublishTimestampDifference(const std_msgs::Float64& timediff, const ros::Publisher& pub) {
  pub.publish(timediff);    
}

bool PointCloudOdometry::LoadCalibrationFromTfTree(){

  ROS_WARN_DELAYED_THROTTLE(2.0, 
                            "Waiting for \'%s\' and \'%s\' to appear in tf_tree...",
                            imu_frame_id_,
                            base_frame_id_);

  tf::StampedTransform imu_T_laser_transform;

  try {
    
    imu_T_laser_listener_.waitForTransform(
      imu_frame_id_,
      base_frame_id_,
      ros::Time(0),
      ros::Duration(2.0));
      
    imu_T_laser_listener_.lookupTransform(
      imu_frame_id_,
      base_frame_id_,
      ros::Time(0),
      imu_T_laser_transform);

    geometry_msgs::TransformStamped imu_T_laser_tmp_msg;

    tf::transformStampedTFToMsg(imu_T_laser_transform, imu_T_laser_tmp_msg);
    
    tf::transformMsgToEigen(imu_T_laser_tmp_msg.transform, I_T_B_);

    B_T_I_ = I_T_B_.inverse();

    ROS_INFO_STREAM("Loaded pose_sensor to imu calibration B_T_L:");

    std::cout << I_T_B_.translation() << std::endl;
    std::cout << I_T_B_.rotation() << std::endl;
    
    I_T_B_q_ = Eigen::Quaterniond(I_T_B_.rotation());
    ROS_INFO("q: x: %.3f, y: %.3f, z: %.3f, w: %.3f", I_T_B_q_.x(), I_T_B_q_.y(), I_T_B_q_.z(), I_T_B_q_.w());

    return true; 

  } 
  
  catch (tf::TransformException ex) {

    ROS_ERROR("%s", ex.what());
    I_T_B_ = Eigen::Affine3d::Identity();
    B_T_I_ = Eigen::Affine3d::Identity();
    return false; 

  }
  
}