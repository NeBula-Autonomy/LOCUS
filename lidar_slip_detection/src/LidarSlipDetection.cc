/*
 * Copyright (c) 2019, NASA Jet Propulsion Laboratory - California
 * Institute of Technology - All rights reserved.
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
 * Authors: Yun Change, Kamak Ebadi ( yunchange@mit.edu,
 * kamak.ebadi@jpl.nasa.gov )
 */

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <lidar_slip_detection/LidarSlipDetection.h>
#include <parameter_utils/ParameterUtils.h>

namespace gu = geometry_utils;
namespace gr = gu::ros;
namespace pu = parameter_utils;

LidarSlipDetection::LidarSlipDetection() {}
LidarSlipDetection::~LidarSlipDetection() {}

bool LidarSlipDetection::Initialize(const ros::NodeHandle& n) {
  // Initialize subscribers
  ros::NodeHandle nl(n);

  if (!LoadParameters(nl)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  lidar_odom_sub_ = nl.subscribe(
      "lio_odom", 10, &LidarSlipDetection::LidarOdometryCallback, this);
  wheel_odom_sub_ = nl.subscribe(
      "wio_odom", 10, &LidarSlipDetection::WheelOdometryCallback, this);
  condition_number_sub_ =
      nl.subscribe("condition_number",
                   10,
                   &LidarSlipDetection::ConditionNumberCallback,
                   this);
  observability_sub_ =
      nl.subscribe("observability_vector",
                   10,
                   &LidarSlipDetection::ObservabilityVectorCallback,
                   this);
  // Initialize publishers
  CreatePublishers(nl);
  return true;
}

bool LidarSlipDetection::LoadParameters(const ros::NodeHandle& n) {
  if (!pu::Get("lidar_slip/slip_threshold", slip_threshold_)) return false;
  if (!pu::Get("lidar_slip/max_power", max_power_)) return false;
  if (!pu::Get("lidar_slip/filter_size", filter_size_)) return false;
  if (!pu::Get("lidar_slip/observability_threshold", observability_threshold_))
    return false;
  if (!pu::Get("lidar_slip/b_use_wio_check", b_use_wio_check_)) return false;
  if (!pu::Get("lidar_slip/b_use_condition_number_check",
               b_use_condition_number_check_))
    return false;
  if (!pu::Get("lidar_slip/b_use_observability_check",
               b_use_observability_check_))
    return false;
  // ROS_INFO_STREAM("lidar slip param is: " << slip_threshold_);
  slip_amount_from_odom_ = 0;  // set to no slip at init (0 slip)
  condition_number_ = 0;       // set to no slip at init (small cond number)
  observability_ = observability_threshold_ * 2;
  // set to no slip (high observability)
  return true;
}

bool LidarSlipDetection::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  slip_detection_from_odom_ =
      nl.advertise<std_msgs::Float64>("slip_detection_from_odom", 10, false);
  lidar_slip_status_pub_ =
      nl.advertise<std_msgs::Bool>("lidar_slip_status", 10, false);
  slip_detection_from_cov_ =
      nl.advertise<std_msgs::Float64>("slip_detection_from_cov", 10, false);

  return true;
}

void LidarSlipDetection::WheelOdometryCallback(const Odometry::ConstPtr& msg) {
  // Callback function for wheel odometry
  PoseCovStamped wheel_current_pose;
  wheel_current_pose.header = msg->header;
  wheel_current_pose.pose = msg->pose;
  last_wo_poses_.push_back(wheel_current_pose);

  // If first entry in last poses more than 1 sec ago
  ros::Duration delta_t =
      wheel_current_pose.header.stamp - last_wo_poses_[0].header.stamp;
  geometry_utils::Transform3 wheel_transform =
      GetTransform(last_wo_poses_[0], wheel_current_pose);
  double x = wheel_transform.translation(0);
  double y = wheel_transform.translation(1);
  double z = wheel_transform.translation(2);
  if (delta_t.toSec() == 0)
    wheel_delta_ = 0.0;
  else
    wheel_delta_ = std::sqrt(x * x + y * y + z * z) / delta_t.toSec();

  // manage filter size (by duration as defined by filter_size_)
  if (delta_t > ros::Duration(filter_size_)) {
    last_wo_poses_.erase(last_wo_poses_.begin());
  }
}

void LidarSlipDetection::LidarOdometryCallback(const Odometry::ConstPtr& msg) {
  // Callback function for lidar odometry
  double slip_amount;
  bool slip_status = false;
  PoseCovStamped lidar_current_pose;
  lidar_current_pose.header = msg->header;
  lidar_current_pose.pose = msg->pose;
  last_lo_poses_.push_back(lidar_current_pose);

  // If first entry in last poses more than 1 sec ago
  ros::Duration delta_t =
      lidar_current_pose.header.stamp - last_lo_poses_[0].header.stamp;
  geometry_utils::Transform3 lidar_transform =
      GetTransform(last_lo_poses_[0], lidar_current_pose);
  double x = lidar_transform.translation(0);
  double y = lidar_transform.translation(1);
  double z = lidar_transform.translation(2);
  if (delta_t.toSec() == 0)
    lidar_delta_ = 0.0;
  else
    lidar_delta_ = std::sqrt(x * x + y * y + z * z) / delta_t.toSec();

  // Calculate slip
  slip_amount_from_odom_ = wheel_delta_ - lidar_delta_;

  // manage filter size (by duration as defined by filter_size_)
  if (delta_t > ros::Duration(filter_size_)) {
    last_lo_poses_.erase(last_lo_poses_.begin());
  }
  PublishLidarSlipAmount(slip_amount_from_odom_, slip_detection_from_odom_);

  // Check for slippage
  bool slip_status_from_wio = true;
  bool slip_status_from_cond_number = true;
  bool slip_status_from_observability = true;

  if (b_use_wio_check_) {
    if (slip_amount_from_odom_ < slip_threshold_) slip_status_from_wio = false;
  }
  if (b_use_condition_number_check_) {
    if (condition_number_ < 8 * exp(max_power_))
      slip_status_from_cond_number = false;
  }
  if (b_use_observability_check_) {
    if (observability_ > observability_threshold_)
      slip_status_from_observability = false;
  }
  if (slip_status_from_observability && slip_status_from_wio 
      && slip_status_from_cond_number)
    slip_status = true;
  PublishLidarSlipStatus(slip_status, lidar_slip_status_pub_);
}

void LidarSlipDetection::ConditionNumberCallback(
    const std_msgs::Float64::ConstPtr& condition_number) {
  // Callback function for condition number
  condition_number_ = condition_number->data;

  PublishConditionNumber(condition_number_, slip_detection_from_cov_);
}

void LidarSlipDetection::ObservabilityVectorCallback(
    const geometry_msgs::Vector3::ConstPtr& msg) {
  observability_ = abs(msg->x);  // Interested in the slip so forward direction
}

geometry_utils::Transform3 LidarSlipDetection::GetTransform(
    const PoseCovStamped first_pose,
    const PoseCovStamped second_pose) {
  // Gets the delta between two pose stamped
  auto pose_first = gr::FromROS(first_pose.pose.pose);
  auto pose_second = gr::FromROS(second_pose.pose.pose);
  auto pose_delta = gu::PoseDelta(pose_first, pose_second);
  return pose_delta;
}

void LidarSlipDetection::PublishLidarSlipAmount(
    const double& slip_detection_odom,
    const ros::Publisher& pub) {
  // Convert slipage value value to ROS format and publish.
  std_msgs::Float64 slip_detection_from_odom;
  slip_detection_from_odom.data = slip_detection_odom;
  pub.publish(slip_detection_from_odom);
}

void LidarSlipDetection::PublishLidarSlipStatus(const bool& slip_status,
                                                const ros::Publisher& pub) {
  // Convert condition number value to ROS format and publish.
  std_msgs::Bool lidar_slip_status;
  lidar_slip_status.data = slip_status;
  pub.publish(lidar_slip_status);
}

void LidarSlipDetection::PublishConditionNumber(
    const double& slip_detection_cov,
    const ros::Publisher& pub) {
  // Convert condition number value to ROS format and publish.
  std_msgs::Float64 slip_detection_from_cov;
  slip_detection_from_cov.data = slip_detection_cov;
  pub.publish(slip_detection_from_cov);
}