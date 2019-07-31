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

  extatt_data_has_been_received_ = false;
  integrated_roll_, integrated_pitch_, integrated_yaw_ = 0, 0, 0;  

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

  if (!pu::Get("extatt/use_extatt_data", use_extatt_data_)) 
    return false;
  if (!pu::Get("extatt/check_extatt_data", check_extatt_data_)) 
    return false;
  if (!pu::Get("extatt/extatt_threshold", extatt_threshold_)) 
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

  rpy_extatt_pub_ = nl.advertise<geometry_msgs::Vector3>("rpy_extatt", 1, false);
  rpy_computed_pub_ = nl.advertise<geometry_msgs::Vector3>("rpy_computed", 1, false);
  timestamp_difference_pub_ = nl.advertise<std_msgs::Float64>("extatt_lidar_ts_diff", 1, false); 

  return true;
}

void PointCloudOdometry::StateEstimateOdometryCallback(
    const nav_msgs::Odometry& msg) {
  // TODO: Andrea: add odometry callback.
}

void PointCloudOdometry::SetExternalAttitude(const geometry_msgs::Quaternion_<std::allocator<void>>& quaternion, const ros::Time& timestamp){  
  
  Eigen::Matrix3f mat3 = Eigen::Quaternionf(float(quaternion.w), float(quaternion.x), float(quaternion.y), float(quaternion.z)).toRotationMatrix();  
  Eigen::Matrix4f mat4 = Eigen::Matrix4f::Identity();
  mat4.block(0,0,3,3) = mat3; 

  // Buffering extatt Data in FIFO Structure
  // An optimal deque size increases speed and decreases computational load when a search in the deque is performed (syncing stage)
  if (extatt_deque_.size()==max_extatt_deque_size_){ 
    extatt_deque_.pop_front();
    extatt_deque_.push_back(extatt_data{mat4, timestamp});
  }
  else{
    extatt_deque_.push_back(extatt_data{mat4, timestamp});
  }

  if (extatt_data_has_been_received_==false){
    extatt_first_attitude_ = mat4; // First time receiving the extatt Data  
    std::cout << "Receiving extatt data for the first time ---> extatt_first_attitude_ now exists!";
    extatt_data_has_been_received_ = true;
  }
}

bool PointCloudOdometry::UpdateEstimate(const PointCloud& points) {

  // As soon as UpdateEstimate is called, a copy of the deque of interest is taken 
  // in order to not pick the wrong i-th element when external producer threads are pushing element into the deque
  std::deque<extatt_data> extatt_deque_copy_ = extatt_deque_; 
  
  // Store input point cloud's time stamp for publishing.
  stamp_.fromNSec(points.header.stamp * 1e3);

  // If this is the first point cloud, store it and wait for anotherinverse.
    if (!initialized_) {
    if (use_extatt_data_==true){
      copyPointCloud(points, *query_); 
      if (extatt_data_has_been_received_ == true){
        extatt_previous_attitude_ = extatt_first_attitude_; // TODO: Check this very first assumption
        initialized_ = true;
        return false;
      }
      else{
        return false;
      }      
    }
    else{
      copyPointCloud(points, *query_);
      initialized_ = true;
      return false;      
    }
  }

  // Deactivate external data fusion if external publisher crashed 
  if(extatt_deque_copy_.size()==min_extatt_deque_size_){
    use_extatt_data_ = false; 
    std::cout<<"External Attitude data provider crashed! Relying on pure ICP now"<< std::endl; 
  }

  if(use_extatt_data_==true){
    // Initialize extatt_current_attitude_ to whatever the attitude of the 0-th element of the deque is
    extatt_current_attitude_ = extatt_deque_copy_[0].internal_extatt_attitude_; 
    // Search for the closest timestamp and get from that particular element the attitude
    float min_ts_diff = 1000;   
    for (int i=0; i<extatt_deque_copy_.size(); ++i) {
          float cur_ts_diff = (extatt_deque_copy_[i].internal_extatt_attitude_timestamp_ - stamp_).toSec();
          // We can accept negative differences (extatt coming from the past in respect to LIDAR) and they've to be as close to zero as possible  
          if (cur_ts_diff<0 && fabs(cur_ts_diff)<fabs(min_ts_diff)){
              extatt_current_attitude_ = extatt_deque_copy_[i].internal_extatt_attitude_; 
              min_ts_diff = cur_ts_diff; 
          }
    }
    // At this point we've picked the correct extatt_current_attitude_ representing the orientation 
    // of the extatt element with the closest Timestamp to the LIDAR Scan
    std_msgs::Float64 extatt_lidar_ts_diff; 
    extatt_lidar_ts_diff.data = min_ts_diff; 
    PublishTimestampDifference(extatt_lidar_ts_diff, timestamp_difference_pub_); 

    // Here we have the correct change in attitude   
    extatt_change_in_attitude_ = extatt_current_attitude_.inverse()*extatt_previous_attitude_;
    Eigen::Matrix4f extatt_change_in_attitude_copy_ = extatt_change_in_attitude_; 
    
    // We now memorize this computed value in the deque 
    extatt_attitude_deque_.push_back(extatt_change_in_attitude_copy_);  

    // Do the check ONLY if check_extatt_data_ flag is set to true
    if (check_extatt_data_==true){
      float max_ts_diff = 0.05; 
      // Set use_extatt_data_ to true only if timestamp difference extatt - LIDAR is below threshold && rpy extatt are below extatt threshold
      if (fabs(min_ts_diff)<fabs(max_ts_diff)){
        use_extatt_data_ = true; 
      }
      else{
          use_extatt_data_ = false; // Check correctness of this approach
          // We could try to weight the extatt Data fusage process basing on the current extatt-LIDAR timestamp difference
          // Or should we go with extatt preintegration and interpolation approach? 
          std::cout << "BAD! ---> " << min_ts_diff << std::endl; 
      }  
    }
  
    // Move current query points (acquired last iteration) to reference points.
    copyPointCloud(*query_, *reference_);

    // Set the incoming point cloud as the query point cloud.
    copyPointCloud(points, *query_);

    // Update extatt
    extatt_previous_attitude_ = extatt_current_attitude_; 

    extatt_deque_.pop_front();

    // Update pose estimate via ICP.
    return UpdateICP();
  }

  else{
    // Move current query points (acquired last iteration) to reference points.
    copyPointCloud(*query_, *reference_);

    // Set the incoming point cloud as the query point cloud.
    copyPointCloud(points, *query_);

    // Update extatt
    extatt_previous_attitude_ = extatt_current_attitude_; 

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

  Eigen::Matrix4f T; 

    if (use_extatt_data_==true){

        T = icp.getFinalTransformation();

        // Compute pure ICP LIDAR rotation 
        Eigen::Matrix3f COMPUTED_ROTATION = T.block(0,0,3,3);
        Eigen::Quaternionf COMPUTED_quaternion(COMPUTED_ROTATION);
        geometry_msgs::Quaternion myComputedOrientation;
        myComputedOrientation.w = COMPUTED_quaternion.w();
        myComputedOrientation.x = COMPUTED_quaternion.x();
        myComputedOrientation.y = COMPUTED_quaternion.y();
        myComputedOrientation.z = COMPUTED_quaternion.z();
        tf::Quaternion myComputedOrientationTf;
        tf::quaternionMsgToTF(myComputedOrientation, myComputedOrientationTf);   // TODO: Do this with tf_conversions::quaternionEigenToTF(cur_extatt_quaternion_double, cur_extatt_quaternion_tf);
        double roll_computed, pitch_computed, yaw_computed;
        tf::Matrix3x3(myComputedOrientationTf).getRPY(roll_computed, pitch_computed, yaw_computed);    
        geometry_msgs::Vector3 rpy_lidar;
        rpy_lidar.x = roll_computed; 
        rpy_lidar.y = pitch_computed; 
        rpy_lidar.z = yaw_computed; 
        PublishRpyComputed(rpy_lidar, rpy_computed_pub_); 

        // Compute pure extatt rotation
        Eigen::Matrix4f extatt_attitude_local_copy_ = extatt_attitude_deque_.front();  
        extatt_attitude_deque_.pop_front();
        std::cout<<"External data queue size: " << extatt_deque_.size()<<std::endl;
        Eigen::Matrix3f cur_extatt_rot = extatt_attitude_local_copy_.block(0,0,3,3);  // Matrix of floats
        Eigen::Matrix3d cur_extatt_rot_double = cur_extatt_rot.cast <double> ();     // Matrix of doubles
        Eigen::Quaterniond cur_extatt_quaternion_double(cur_extatt_rot_double);
        tf::Quaternion cur_extatt_quaternion_tf;
        geometry_msgs::Quaternion cur_extatt_quaternion_msg;
        cur_extatt_quaternion_msg.w = cur_extatt_quaternion_double.w();
        cur_extatt_quaternion_msg.x = cur_extatt_quaternion_double.x();
        cur_extatt_quaternion_msg.y = cur_extatt_quaternion_double.y();
        cur_extatt_quaternion_msg.z = cur_extatt_quaternion_double.z();
        tf::quaternionMsgToTF(cur_extatt_quaternion_msg, cur_extatt_quaternion_tf);   // TODO: Do this with tf_conversions::quaternionEigenToTF(cur_extatt_quaternion_double, cur_extatt_quaternion_tf);
        double roll_extatt, pitch_extatt, yaw_extatt;
        tf::Matrix3x3(cur_extatt_quaternion_tf).getRPY(roll_extatt, pitch_extatt, yaw_extatt);
        geometry_msgs::Vector3 rpy_extatt;
        rpy_extatt.x = roll_extatt; 
        rpy_extatt.y = pitch_extatt; 
        rpy_extatt.z = yaw_extatt; 
        PublishRpyExtatt(rpy_extatt, rpy_extatt_pub_); 

        // Get RPY from external data source 
        float out_roll = -float(roll_extatt);
        float out_pitch = -float(pitch_extatt);
        float out_yaw = -float(yaw_extatt);

        Eigen::Matrix3f OUTPUT_ROTATION; 
        OUTPUT_ROTATION = Eigen::AngleAxisf(out_roll, Eigen::Vector3f::UnitX())
          * Eigen::AngleAxisf(out_pitch, Eigen::Vector3f::UnitY())
          * Eigen::AngleAxisf(out_yaw, Eigen::Vector3f::UnitZ()); 

        T.block(0,0,3,3) = OUTPUT_ROTATION;
      
        std::cout << "extatt ON" << std::endl;  
  }
  else{
        T = icp.getFinalTransformation();
        std::cout << "extatt OFF" << std::endl;  
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

void PointCloudOdometry::PublishRpyExtatt(const geometry_msgs::Vector3& rpy, const ros::Publisher& pub) {
  pub.publish(rpy);    
}

void PointCloudOdometry::PublishRpyComputed(const geometry_msgs::Vector3& rpy, const ros::Publisher& pub) {
  pub.publish(rpy);    
}
                                                 
void PointCloudOdometry::PublishTimestampDifference(const std_msgs::Float64& timediff, const ros::Publisher& pub) {
  pub.publish(timediff);    
}
