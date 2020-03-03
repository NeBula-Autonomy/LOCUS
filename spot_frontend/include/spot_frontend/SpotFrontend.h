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

#ifndef LO_FRONTEND_SPOT_FRONTEND_H
#define LO_FRONTEND_SPOT_FRONTEND_H

#include <chrono>
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
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/message_filter.h>
#include <tf2_ros/message_filter.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

class SpotFrontend {

public:

  typedef pcl::PointXYZI Point;
  typedef pcl::PointCloud<Point> PointCloud;    

  SpotFrontend();
  ~SpotFrontend();

  bool Initialize(const ros::NodeHandle& n, bool from_log);

private:

  std::string name_;
  std::string fixed_frame_id_; 
  std::string base_frame_id_;
  std::string bd_odom_frame_id_;
  const std::string tf_buffer_authority_;   

  bool b_verbose_;

  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n, bool from_log);
  bool RegisterLogCallbacks(const ros::NodeHandle& n);
  bool RegisterOnlineCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  ros::Subscriber odometry_sub_;  
  tf2_ros::Buffer odometry_buffer_;
  message_filters::Subscriber<PointCloud> lidar_sub_;
  tf2_ros::MessageFilter<PointCloud> *lidar_odometry_filter_;
  ros::Subscriber lidar_ros_sub_;

  int odom_queue_size_; 
  int lidar_queue_size_;  
  int odometry_buffer_size_limit_;

  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg);
  void LidarCallback(const PointCloud::ConstPtr& msg);

  ros::Publisher base_frame_pcld_pub_; 

  template <typename T>
  int CheckBufferSize(const T& buffer) const;

  double translation_threshold_kf_;
  double rotation_threshold_kf_;
  bool b_add_first_scan_to_key_;

  gtsam::Pose3 ToGtsam(const geometry_utils::Transform3& pose) const;   
  geometry_utils::Transform3 last_keyframe_pose_;      
  
  PointCloudFilter filter_;
  PointCloudOdometry odometry_;
  PointCloudLocalization localization_; 
  PointCloudMapper mapper_;   

  bool b_publish_map_;
  int counter_;
  int map_publishment_meters_;
  
  bool b_pcld_received_;
  int pcld_seq_prev_;

  PointCloud::Ptr msg_filtered_;
  PointCloud::Ptr msg_transformed_;
  PointCloud::Ptr msg_neighbors_;
  PointCloud::Ptr msg_base_;
  PointCloud::Ptr msg_fixed_;
  PointCloud::Ptr mapper_unused_fixed_;
  PointCloud::Ptr mapper_unused_out_;

  /*--------------
  Data integration 
  --------------*/

  bool SetDataIntegrationMode();
  int data_integration_mode_;
  int max_number_of_calls_;

  bool b_use_odometry_integration_;
  bool b_odometry_has_been_received_;
  int odometry_number_of_calls_;
  tf::Transform odometry_pose_previous_;
  tf::Transform GetOdometryDelta(const tf::Transform& odometry_pose) const;

  /*-----------------
  Open space detector
  ------------------*/

  bool b_is_open_space_;
  int number_of_points_open_space_;

  /*--------------------
  Flat Ground Assumption
  --------------------*/

  ros::Subscriber fga_sub_;
  void FlatGroundAssumptionCallback(const std_msgs::Bool& bool_msg);
    
};

#endif 