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

#ifndef LO_FRONTEND_LO_FRONTEND_H
#define LO_FRONTEND_LO_FRONTEND_H

#include <math.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>
#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_odometry/PointCloudOdometry.h>
#include <point_cloud_localization/PointCloudLocalization.h>
#include <point_cloud_mapper/PointCloudMapper.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <core_msgs/PoseAndScan.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <core_msgs/PoseAndScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class LoFrontend {

public:

  typedef pcl::PointXYZI Point;
  typedef pcl::PointCloud<Point> PointCloud;
  typedef sensor_msgs::Imu Imu;
  typedef nav_msgs::Odometry Odometry;
  typedef geometry_msgs::PoseStamped PoseStamped; 
  typedef std::map<double, Imu> ImuBuffer;       
  typedef std::map<double, Odometry> OdometryBuffer;         
  typedef std::map<double, PoseStamped> PoseStampedBuffer;   
  typedef Imu::ConstPtr ImuConstPtr;
  typedef Odometry::ConstPtr OdometryConstPtr;
  typedef PoseStamped::ConstPtr PoseStampedConstPtr;

  LoFrontend();
  ~LoFrontend();

  bool Initialize(const ros::NodeHandle& n, bool from_log);

private:

  std::string name_;
  bool b_verbose_;

  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
  bool RegisterLogCallbacks(const ros::NodeHandle& n);
  bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  void ImuCallback(const ImuConstPtr& imu_msg);
  void OdometryCallback(const OdometryConstPtr& odometry_msg);
  void PoseStampedCallback(const PoseStampedConstPtr& pose_stamped_msg);
  void PointCloudCallback(const PointCloud::ConstPtr& msg);

  ros::Subscriber pcld_sub_;  
  ros::Subscriber imu_sub_;   
  ros::Subscriber odom_sub_;  
  ros::Subscriber pose_sub_;    

  ros::Publisher base_frame_pcld_pub_; 

  ImuBuffer imu_buffer_;
  OdometryBuffer odometry_buffer_;
  PoseStampedBuffer pose_stamped_buffer_;
  
  int imu_buffer_size_limit_; 
  int odometry_buffer_size_limit_;
  int pose_stamped_buffer_size_limit_;

  template <typename T1, typename T2>
  bool InsertMsgInBuffer(const T1& msg, T2& buffer);

  template <typename T>
  int CheckBufferSize(const T& buffer) const;

  template <typename T1, typename T2>
  bool GetMsgAtTime(const ros::Time& stamp, T1& msg, T2& buffer) const;         

  double translation_threshold_kf_;
  double rotation_threshold_kf_;
  bool b_add_first_scan_to_key_;

  gtsam::Pose3 ToGtsam(const geometry_utils::Transform3& pose) const;   
  geometry_utils::Transform3 last_keyframe_pose_; 
  ros::Time last_pcld_stamp_;        

  std::string fixed_frame_id_; 
  std::string base_frame_id_; 
  std::string imu_frame_id_;
  
  bool LoadCalibrationFromTfTree();
  tf::TransformListener imu_T_base_listener_;
  Eigen::Affine3d I_T_B_;    
  Eigen::Affine3d B_T_I_; 
  Eigen::Quaterniond I_T_B_q_;   

  // IMU Frontend Integration   
  double ts_threshold_;
  Eigen::Quaterniond GetImuQuaternion(const Imu& imu_msg);
  bool b_convert_imu_to_base_link_frame_;
  bool b_use_imu_integration_;
  int imu_number_of_calls_;
  int imu_max_number_of_calls_;

  // ODOMETRY Frontend Integration 
  bool b_use_odometry_integration_;
  int odometry_number_of_calls_;
  int odometry_max_number_of_calls_;
  bool b_odometry_has_been_received_;
  tf::Transform odometry_pose_previous_;
  tf::Transform GetOdometryDelta(const Odometry& odometry_msg) const; 
  
  // Class objects
  PointCloudFilter filter_;
  PointCloudOdometry odometry_;
  PointCloudLocalization localization_; 
  PointCloudMapper mapper_;   

  // Map publishment 
  int counter_;
  bool b_publish_map_;
  int map_publishment_meters_;
  
  bool b_pcld_received_;
  int pcld_seq_prev_;

  // Storages - TODO: This can be optimized
  PointCloud::Ptr msg_filtered_;
  PointCloud::Ptr msg_transformed_;
  PointCloud::Ptr msg_neighbors_;
  PointCloud::Ptr msg_base_;
  PointCloud::Ptr msg_fixed_;
  PointCloud::Ptr mapper_unused_fixed_;
  PointCloud::Ptr mapper_unused_out_;

  // Load queue sizes as params 
  int imu_queue_size_; 
  int odom_queue_size_; 
  int pose_queue_size_; 
  int lidar_queue_size_; 

};

#endif 