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

#include <lo_frontend/LoFrontend.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

LoFrontend::LoFrontend(): 
  b_add_first_scan_to_key_(true),
  translation_threshold_kf_(1.0),
  rotation_threshold_kf_(1.0), 
  counter_(0), 
  b_verbose_(false),
  b_pcld_received_(false),
  msg_filtered_(new PointCloud()),
  msg_transformed_(new PointCloud()),
  msg_neighbors_(new PointCloud()),
  msg_base_(new PointCloud()),
  msg_fixed_(new PointCloud()), 
  mapper_unused_fixed_(new PointCloud()),
  mapper_unused_out_(new PointCloud()), 
  imu_number_of_calls_(0), 
  b_odometry_has_been_received_(false) {}

LoFrontend::~LoFrontend() {}

bool LoFrontend::Initialize(const ros::NodeHandle& n, bool from_log) {
  ROS_INFO("LoFrontend - Initialize");  
  name_ = ros::names::append(n.getNamespace(), "lo_frontend");  
  if (!filter_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud filter.", name_.c_str());
    return false;
  }  
  if (!odometry_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud odometry.", name_.c_str());
    return false;
  }  
  if (!localization_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize localization.", name_.c_str());
    return false;
  }  
  if (!mapper_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize mapper.", name_.c_str());
    return false;
  }  
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }  
  if (!RegisterCallbacks(n, from_log)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }  
  LoadCalibrationFromTfTree();
  return true;  
}

bool LoFrontend::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("LoFrontend - LoadParameters");  
  if (!pu::Get("b_verbose", b_verbose_))
    return false;
  if (!pu::Get("b_publish_map", b_publish_map_))
    return false;
  if (!pu::Get("translation_threshold_kf", translation_threshold_kf_))
    return false;
  if (!pu::Get("rotation_threshold_kf", rotation_threshold_kf_))
    return false;
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  if (!pu::Get("frame_id/base", base_frame_id_))
    return false;
  if (!pu::Get("frame_id/imu", imu_frame_id_)) 
    return false;
  if (!pu::Get("frame_conversions/b_convert_imu_to_base_link_frame", b_convert_imu_to_base_link_frame_)) 
    return false;
  if (!pu::Get("buffers/imu_buffer_size_limit", imu_buffer_size_limit_))
    return false;
  if (!pu::Get("buffers/odometry_buffer_size_limit", odometry_buffer_size_limit_))
    return false;
  if (!pu::Get("buffers/pose_stamped_buffer_size_limit", pose_stamped_buffer_size_limit_))
    return false;
  if (!pu::Get("syncing/ts_threshold", ts_threshold_))
    return false;  
  if(!pu::Get("imu_integration/b_use_imu_integration", b_use_imu_integration_))
    return false;
  if(!pu::Get("imu_integration/imu_max_number_of_calls", imu_max_number_of_calls_))
    return false;
  if(!pu::Get("odometry_integration/b_use_odometry_integration", b_use_odometry_integration_))
    return false;
  if(!pu::Get("odometry_integration/odometry_max_number_of_calls", odometry_max_number_of_calls_))
    return false;
  if(!pu::Get("queue_sizes/imu_queue_size", imu_queue_size_))
    return false;
  if(!pu::Get("queue_sizes/odom_queue_size", odom_queue_size_))
    return false;
  if(!pu::Get("queue_sizes/pose_queue_size", pose_queue_size_))
    return false;
  if(!pu::Get("queue_sizes/lidar_queue_size", lidar_queue_size_))
    return false;
  if(!pu::Get("map_publishment/meters", map_publishment_meters_))
    return false;
  return true;
}

bool LoFrontend::RegisterCallbacks(const ros::NodeHandle& n, bool from_log) {
  ROS_INFO("LoFrontend - RegisterCallbacks");  
  ros::NodeHandle nl(n);  
  if (from_log)
    return RegisterLogCallbacks(n);
  else
    return RegisterOnlineCallbacks(n);    
}

bool LoFrontend::RegisterLogCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("LoFrontend - RegisterLogCallbacks");  
  ROS_INFO("%s: Registering log callbacks.", name_.c_str());
  return CreatePublishers(n);
}

bool LoFrontend::RegisterOnlineCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("LoFrontend - RegisterOnlineCallbacks");  
  ROS_INFO("%s: Registering online callbacks.", name_.c_str());  
  ros::NodeHandle nl(n);      
  imu_sub_ = nl.subscribe("IMU_TOPIC", imu_queue_size_, &LoFrontend::ImuCallback, this);
  odom_sub_ = nl.subscribe("ODOM_TOPIC", odom_queue_size_, &LoFrontend::OdometryCallback, this); 
  pose_sub_ = nl.subscribe("POSE_TOPIC", pose_queue_size_, &LoFrontend::PoseStampedCallback, this);  
  pcld_sub_ = nl.subscribe("LIDAR_TOPIC", lidar_queue_size_, &LoFrontend::PointCloudCallback, this);
  return CreatePublishers(n);
}

bool LoFrontend::CreatePublishers(const ros::NodeHandle& n) {
  ROS_INFO("LoFrontend - CreatePublishers");  
  ros::NodeHandle nl(n);  
  base_frame_pcld_pub_ = nl.advertise<PointCloud>("base_frame_point_cloud", 10, false);
  return true;
}

void LoFrontend::ImuCallback(const ImuConstPtr& imu_msg) {
  if (b_verbose_) ROS_INFO("LoFrontend - ImuCallback"); 
  if (CheckBufferSize(imu_buffer_) > imu_buffer_size_limit_) {
      imu_buffer_.erase(imu_buffer_.begin());
  }   
  if (!InsertMsgInBuffer(imu_msg, imu_buffer_)) {
      ROS_WARN("LoFrontend - ImuCallback - Unable to store message in buffer");
  }
}

void LoFrontend::OdometryCallback(const OdometryConstPtr& odometry_msg) {
  if (b_verbose_) ROS_INFO("LoFrontend - OdometryCallback"); 
  if (CheckBufferSize(odometry_buffer_) > odometry_buffer_size_limit_) {
      odometry_buffer_.erase(odometry_buffer_.begin());
  }   
  if (!InsertMsgInBuffer(odometry_msg, odometry_buffer_)) {
      ROS_WARN("LoFrontend - OdometryCallback - Unable to store message in buffer");
  }
}

void LoFrontend::PoseStampedCallback(const PoseStampedConstPtr& pose_stamped_msg) {
  if (b_verbose_) ROS_INFO("LoFrontend - PoseStampedCallback"); 
  if (CheckBufferSize(pose_stamped_buffer_) > pose_stamped_buffer_size_limit_) {
      pose_stamped_buffer_.erase(pose_stamped_buffer_.begin());
  }   
  if (!InsertMsgInBuffer(pose_stamped_msg, pose_stamped_buffer_)) {
      ROS_WARN("LoFrontend - PoseStampedCallback - Unable to store message in buffer");
  }
}

Eigen::Quaterniond LoFrontend::GetImuQuaternion(const Imu& imu_msg) {
  if (b_verbose_) ROS_INFO("LoFrontend - GetImuQuaternion"); 
  Eigen::Quaterniond imu_quaternion = Eigen::Quaterniond(double(imu_msg.orientation.w), 
                                                         double(imu_msg.orientation.x),  
                                                         double(imu_msg.orientation.y), 
                                                         double(imu_msg.orientation.z));
  if (b_convert_imu_to_base_link_frame_) {
    imu_quaternion = I_T_B_q_*imu_quaternion*I_T_B_q_.inverse();
  }
  return imu_quaternion;
}

bool LoFrontend::LoadCalibrationFromTfTree() {
  ROS_INFO("LoFrontend - LoadCalibrationFromTfTree");  
  ROS_WARN_DELAYED_THROTTLE(2.0, 
                          "Waiting for \'%s\' and \'%s\' to appear in tf_tree...",
                          imu_frame_id_,
                          base_frame_id_);
  tf::StampedTransform imu_T_base_transform;
  try {   
      imu_T_base_listener_.waitForTransform(
          imu_frame_id_,
          base_frame_id_,
          ros::Time(0),
          ros::Duration(2.0));
    
      imu_T_base_listener_.lookupTransform(
          imu_frame_id_,
          base_frame_id_,
          ros::Time(0),
          imu_T_base_transform);

      geometry_msgs::TransformStamped imu_T_base_tmp_msg;
      tf::transformStampedTFToMsg(imu_T_base_transform, imu_T_base_tmp_msg);        
      tf::transformMsgToEigen(imu_T_base_tmp_msg.transform, I_T_B_);
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

gtsam::Pose3 LoFrontend::ToGtsam(const geometry_utils::Transform3& pose) const {
  if(b_verbose_) ROS_INFO("LoFrontend - ToGtsam");
  gtsam::Vector3 t;
  t(0) = pose.translation(0);
  t(1) = pose.translation(1);
  t(2) = pose.translation(2);
  gtsam::Rot3 r(pose.rotation(0, 0),
                pose.rotation(0, 1),
                pose.rotation(0, 2),
                pose.rotation(1, 0),
                pose.rotation(1, 1),
                pose.rotation(1, 2),
                pose.rotation(2, 0),
                pose.rotation(2, 1),
                pose.rotation(2, 2));
  return gtsam::Pose3(r, t);
}

template <typename T1, typename T2>
bool LoFrontend::InsertMsgInBuffer(const T1& msg, T2& buffer) {
  if(b_verbose_) ROS_INFO("LoFrontend - InsertMsgInBuffer");  
  auto initial_size = buffer.size();    
  auto current_time = msg->header.stamp.toSec();    
  buffer.insert({current_time, *msg});
  auto final_size = buffer.size();    
  if (final_size == (initial_size+1)) {
    return true;
  }
  else {
    return false; 
  }
}

template <typename T>
int LoFrontend::CheckBufferSize(const T& buffer) const {
  if(b_verbose_) ROS_INFO("LoFrontend - ChechBufferSize");    
  return buffer.size();
}

template <typename T1, typename T2>
bool LoFrontend::GetMsgAtTime(const ros::Time& stamp, T1& msg, T2& buffer) const {
  if (b_verbose_) ROS_INFO("LoFrontend - GetMsgAtTime"); 
  if (buffer.size() == 0) {
      return false;
  }
  auto itrTime = buffer.lower_bound(stamp.toSec());
  auto time2 = itrTime->first;
  double time_diff;    
  if (itrTime == buffer.begin()) {
      msg = itrTime->second;
      time_diff = itrTime->first - stamp.toSec();
      ROS_WARN("itrTime points to buffer begin");
  }
  else if (itrTime == buffer.end()) {
      itrTime--;
      msg = itrTime->second;
      time_diff = stamp.toSec() - itrTime->first;
      ROS_WARN("itrTime points to buffer end");
  }
  else {
      double time1 = std::prev(itrTime, 1)->first;
      if (time2 - stamp.toSec() < stamp.toSec() - time1) {
          msg = itrTime->second;
          time_diff = time2 - stamp.toSec();
      } 
      else {
          msg = std::prev(itrTime, 1)->second;
          time_diff = stamp.toSec() - time1;
      }
  }
  if (fabs(time_diff) > ts_threshold_) { 
      ROS_WARN_STREAM("Time difference is "
                      << time_diff << "s, threshold is: " << ts_threshold_ << ", returning false");
      return false;
  }     
return true; 
}

tf::Transform LoFrontend::GetOdometryDelta(const Odometry& odometry_msg) {
  tf::Transform odometry_pose;
  tf::poseMsgToTF(odometry_msg.pose.pose, odometry_pose);
  auto odometry_delta = odometry_pose.inverseTimes(odometry_pose_previous_);
  return odometry_delta;
}

void LoFrontend::PointCloudCallback(const PointCloud::ConstPtr& msg) {  
  if (b_verbose_) ROS_INFO("LoFrontend - PointCloudCallback"); 

  if(!b_pcld_received_) {
    pcld_seq_prev_ = msg->header.seq;
    b_pcld_received_ = true;
  }
  else {
    if(msg->header.seq!=pcld_seq_prev_+1) {
      ROS_WARN("Lidar scan dropped");
    }
    pcld_seq_prev_ = msg->header.seq;
  }

  auto msg_stamp = msg->header.stamp;
  ros::Time stamp = pcl_conversions::fromPCL(msg_stamp);

  /*
  TODO: Wrap these in single logic ------------------------------------------------------------------------------------------------
  */

  if(b_use_imu_integration_) {
    Imu imu_msg;
    if(!GetMsgAtTime(stamp, imu_msg, imu_buffer_)) {
      ROS_WARN("Unable to retrieve imu_msg from imu_buffer_ given Lidar timestamp");
      imu_number_of_calls_++;
      if (imu_number_of_calls_ > imu_max_number_of_calls_) {
        // TODO: Robustify with consecutiveness-check (unified method to handle IMU crash at any time)
        ROS_WARN("Deactivating imu_integration in LoFrontend as imu_number_of_calls > imu_max_number_of_calls");
        b_use_imu_integration_ = false;
      }
      return;
    }
    odometry_.SetImuQuaternion(GetImuQuaternion(imu_msg));
  }

  if (b_use_odometry_integration_) {
    Odometry odometry_msg;
    if(!GetMsgAtTime(stamp, odometry_msg, odometry_buffer_)) {
      ROS_WARN("Unable to retrieve odometry_msg from odometry_buffer_ given Lidar timestamp");
      odometry_number_of_calls_++;
      if (odometry_number_of_calls_ > odometry_max_number_of_calls_) {
        // TODO: Robustify with consecutiveness-check (unified method to handle ODOMETRY crash at any time)
        ROS_WARN("Deactivating odometry_integration in LoFrontend as odometry_number_of_calls > odometry_max_number_of_calls");
        b_use_odometry_integration_ = false;
      }
      return;
    }
    if (!b_odometry_has_been_received_) {
      ROS_INFO("Receiving odometry for the first time");
      tf::poseMsgToTF(odometry_msg.pose.pose, odometry_pose_previous_);
      b_odometry_has_been_received_= true;
      return;
    }
    // odometry_.SetOdometryDelta(GetOdometryDelta(odometry_msg)); 
  }

  /*
  ---------------------------------------------------------------------------------------------------------------------------------
  */
  
  filter_.Filter(msg, msg_filtered_);
  odometry_.SetLidar(*msg_filtered_);
  
  if (!odometry_.UpdateEstimate()) {
     b_add_first_scan_to_key_ = true;
  }

  if (b_add_first_scan_to_key_) {
    localization_.TransformPointsToFixedFrame(*msg, msg_transformed_.get());
    mapper_.InsertPoints(msg_transformed_, mapper_unused_fixed_.get());
    localization_.UpdateTimestamp(stamp);
    localization_.PublishPoseNoUpdate();
    b_add_first_scan_to_key_ = false;
    last_keyframe_pose_ = localization_.GetIntegratedEstimate();
    return;
  }  

  localization_.MotionUpdate(odometry_.GetIncrementalEstimate());
  localization_.TransformPointsToFixedFrame(*msg, msg_transformed_.get());
  mapper_.ApproxNearestNeighbors(*msg_transformed_, msg_neighbors_.get());   
  localization_.TransformPointsToSensorFrame(*msg_neighbors_, msg_neighbors_.get());
  localization_.MeasurementUpdate(msg_filtered_, msg_neighbors_, msg_base_.get());
  geometry_utils::Transform3 current_pose = localization_.GetIntegratedEstimate();
  gtsam::Pose3 delta = ToGtsam(geometry_utils::PoseDelta(last_keyframe_pose_, current_pose));
  
  if (delta.translation().norm()>translation_threshold_kf_ ||
      fabs(2*acos(delta.rotation().toQuaternion().w()))>rotation_threshold_kf_) {
    if(b_verbose_) ROS_INFO_STREAM("Adding to map with translation " << delta.translation().norm() << " and rotation " << 2*acos(delta.rotation().toQuaternion().w())*180.0/M_PI << " deg");
    localization_.MotionUpdate(gu::Transform3::Identity());
    localization_.TransformPointsToFixedFrame(*msg, msg_fixed_.get());
    mapper_.InsertPoints(msg_fixed_, mapper_unused_out_.get());
    if(b_publish_map_) {
      counter_++;   
      if (counter_==map_publishment_meters_) { 
        ROS_INFO("Publishing map");
        mapper_.PublishMap();
        counter_ = 0;
      }
    } 
    last_keyframe_pose_ = current_pose;
  }

  if (base_frame_pcld_pub_.getNumSubscribers() != 0) {
    PointCloud base_frame_pcld = *msg;
    base_frame_pcld.header.frame_id = base_frame_id_;
    base_frame_pcld_pub_.publish(base_frame_pcld);
  }  
  
}