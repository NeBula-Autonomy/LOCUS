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

#include <spot_frontend/SpotFrontend.h>

namespace pu = parameter_utils;
namespace gu = geometry_utils;

SpotFrontend::SpotFrontend(): 
  b_add_first_scan_to_key_(true),
  counter_(0), 
  b_pcld_received_(false),
  msg_filtered_(new PointCloud()),
  msg_transformed_(new PointCloud()),
  msg_neighbors_(new PointCloud()),
  msg_base_(new PointCloud()),
  msg_fixed_(new PointCloud()), 
  mapper_unused_fixed_(new PointCloud()),
  mapper_unused_out_(new PointCloud()), 
  b_use_odometry_integration_(false), 
  odometry_number_of_calls_(0), 
  b_odometry_has_been_received_(false),
  b_is_open_space_(false), 
  tf_buffer_authority_("transform_odometry"),
  odometry_frame_("odometry_frame"), 
  lidar_frame_("lidar_frame"), 
  inertial_frame_("inertial_frame") {}

SpotFrontend::~SpotFrontend() {}

bool SpotFrontend::Initialize(const ros::NodeHandle& n, bool from_log) {
  ROS_INFO("SpotFrontend - Initialize");  
  name_ = ros::names::append(n.getNamespace(), "spot_frontend");  
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
  if (!SetDataIntegrationMode()) {
    ROS_ERROR("Failed to set data integration mode");
    return false;
  }
  if (!RegisterCallbacks(n, from_log)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  return true;
}

bool SpotFrontend::LoadParameters(const ros::NodeHandle& n) {
  ROS_INFO("SpotFrontend - LoadParameters");  
  if (!pu::Get("b_verbose", b_verbose_))
    return false;
  if (!pu::Get("translation_threshold_kf", translation_threshold_kf_))
    return false;
  if (!pu::Get("rotation_threshold_kf", rotation_threshold_kf_))
    return false;
  if (!pu::Get("number_of_points_open_space", number_of_points_open_space_))
    return false;
  if(!pu::Get("map_publishment/meters", map_publishment_meters_))
    return false;
  if (!pu::Get("map_publishment/b_publish_map", b_publish_map_))
    return false;  
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  if (!pu::Get("frame_id/base", base_frame_id_))
    return false;
  if (!pu::Get("queues/lidar_queue_size", lidar_queue_size_)) 
    return false;
  if (!pu::Get("queues/odom_queue_size", odom_queue_size_))
    return false;
  if (!pu::Get("buffers/odometry_buffer_size_limit", odometry_buffer_size_limit_))
    return false;
  if(!pu::Get("data_integration/mode", data_integration_mode_))
    return false;
  if(!pu::Get("data_integration/max_number_of_calls", max_number_of_calls_))
    return false;
  return true;
}

bool SpotFrontend::SetDataIntegrationMode() {  
  ROS_INFO("SpotFrontend - SetDataIntegrationMode");  
  switch (data_integration_mode_) {
    case 0:
      ROS_INFO("No integration requested");
      break;
    case 3: 
      ROS_INFO("Odometry integration requested");
      b_use_odometry_integration_ = true;
      odometry_.EnableOdometryIntegration();
      break;
    default:
      ROS_ERROR("Default case to be handled");
      return false;
      break;   
  }
  return true;
}

bool SpotFrontend::RegisterCallbacks(const ros::NodeHandle& n, bool from_log) {
  ROS_INFO("SpotFrontend - RegisterCallbacks");  
  ros::NodeHandle nl(n);  
  if (from_log)
    return RegisterLogCallbacks(n);
  else
    return RegisterOnlineCallbacks(n);    
}

bool SpotFrontend::RegisterLogCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("SpotFrontend - RegisterLogCallbacks");  
  ROS_INFO("%s: Registering log callbacks.", name_.c_str());
  return CreatePublishers(n);
}

bool SpotFrontend::RegisterOnlineCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("SpotFrontend - RegisterOnlineCallbacks");  
  ROS_INFO("%s: Registering online callbacks.", name_.c_str());  
  ros::NodeHandle nl(n);
  // New way to register subscribers and callbacks with message filters
  odometry_sub_ = nl.subscribe("ODOMETRY_TOPIC", odom_queue_size_, &SpotFrontend::OdometryCallback, this); 
  
  lidar_sub_.subscribe(nl, "LIDAR_TOPIC", lidar_queue_size_);
  lidar_odometry_filter_ = new tf2_ros::MessageFilter<PointCloud>(lidar_sub_, odometry_buffer_, "spot1/odom", 10, nl); 
  lidar_odometry_filter_->registerCallback(boost::bind(&SpotFrontend::LidarCallback, this, _1));
  
  /*
  // TODO: For now we assume that VO integration is baseline when running spot
  lidar_sub_ = nl.subscribe("LIDAR_TOPIC", lidar_queue_size_, &SpotFrontend::LidarCallbackOld, this);
  if (b_use_odometry_integration_) odometry_sub_ = nl.subscribe("ODOMETRY_TOPIC", odom_queue_size_, &SpotFrontend::OdometryCallback, this);   
  */
  return CreatePublishers(n);
}

bool SpotFrontend::CreatePublishers(const ros::NodeHandle& n) {
  ROS_INFO("SpotFrontend - CreatePublishers");  
  ros::NodeHandle nl(n);  
  base_frame_pcld_pub_ = nl.advertise<PointCloud>("base_frame_point_cloud", 10, false);
  return true;
}

void SpotFrontend::OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg) {
  ROS_INFO("SpotFrontend - OdometryCallback"); 
  geometry_msgs::TransformStamped odometry;
  geometry_msgs::Vector3 t;
  t.x = odometry_msg->pose.pose.position.x;
  t.y = odometry_msg->pose.pose.position.y;
  t.z = odometry_msg->pose.pose.position.z; 
  odometry.transform.translation = t;
  odometry.transform.rotation = odometry_msg->pose.pose.orientation;
  odometry.header = odometry_msg->header;
  odometry.header.frame_id = "spot1/odom";
  odometry.child_frame_id = "spot1/base_link";
  odometry_buffer_.setTransform(odometry, tf_buffer_authority_, false); 
}

void SpotFrontend::LidarCallback(const PointCloud::ConstPtr& msg) {  
  ROS_INFO("LoFrontend - LidarCallback"); 

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

  auto number_of_points = msg->width;
  if (number_of_points > number_of_points_open_space_) b_is_open_space_ = true;
  else b_is_open_space_ = false;  
  
  auto msg_stamp = msg->header.stamp;
  ros::Time stamp = pcl_conversions::fromPCL(msg_stamp);

  // VO integration --------------------------------------------------------------------
  if (b_use_odometry_integration_) {
    auto t = odometry_buffer_.lookupTransform("spot1/odom", "spot1/base_link", stamp);
    // TODO: Deactivate if VO dies
    // Convert geometry_msgs::TransformStamped to tf::Transform
    tf::Transform tf_transform;
    tf::Vector3 tf_translation;
    tf::Quaternion tf_quaternion;
    tf::vector3MsgToTF(t.transform.translation, tf_translation);
    tf::quaternionMsgToTF(t.transform.rotation, tf_quaternion);
    tf_transform.setOrigin(tf_translation);
    tf_transform.setRotation(tf_quaternion);
    if (!b_odometry_has_been_received_) {
      ROS_INFO("Receiving odometry for the first time");
      // Store odometry_pose_previous 
      odometry_pose_previous_ = tf_transform;  
      b_odometry_has_been_received_= true;
      return;
    }
    odometry_.SetOdometryDelta(GetOdometryDelta(tf_transform)); 
    odometry_pose_previous_ = tf_transform;
  }
  // ---------------------------------------------------------------------------------
 
  filter_.Filter(msg, msg_filtered_, b_is_open_space_);
  odometry_.SetLidar(*msg_filtered_);

  ROS_INFO("Calling UpdateEstimate on odometry_"); 
  
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

tf::Transform SpotFrontend::GetOdometryDelta(const tf::Transform& odometry_pose) const {
  return odometry_pose_previous_.inverseTimes(odometry_pose);;
}

template <typename T>
int SpotFrontend::CheckBufferSize(const T& buffer) const {
  if(b_verbose_) ROS_INFO("SpotFrontend - ChechBufferSize");    
  return buffer.size();
}

gtsam::Pose3 SpotFrontend::ToGtsam(const geometry_utils::Transform3& pose) const {
  if(b_verbose_) ROS_INFO("SpotFrontend - ToGtsam");
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
