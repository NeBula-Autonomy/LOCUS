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
 * Authors: Yun Change, Kamak Ebadi ( yunchange@mit.edu, kamak.ebadi@jpl.nasa.gov )
 */

#include <lidar_slip_detection/LidarSlipDetection.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
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
  condition_number_sub_ = nl.subscribe(
    "condition_number", 10, &LidarSlipDetection::ConditionNumberCallback, this);
  // Initialize publishers
  CreatePublishers(nl);
  // Initialize wheel and lidar poses to zero
  InitializePose(lidar_last_pose_);
  InitializePose(wheel_last_pose_);
  return true;
}

bool LidarSlipDetection::LoadParameters(const ros::NodeHandle& n) {
  if (!pu::Get("lidar_slip/slip_threshold", slip_threshold_)) return false;
    ROS_INFO_STREAM("lidar slip param is: " << slip_threshold_);
  return true;
}


bool LidarSlipDetection::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  lidar_slip_amount_pub_ = nl.advertise<std_msgs::Float64>("lidar_slip_amount", 10, false);
  lidar_slip_status_pub_ = nl.advertise<std_msgs::Bool>("lidar_slip_status", 10, false);
  // avg_condition_number_pub_ = nl.advertise<std_msgs::Float64>("average_condition_number", 10, false);

  return true;
}

void LidarSlipDetection::WheelOdometryCallback(const Odometry::ConstPtr& msg) {
  // Callback function for wheel odometry
  PoseCovStamped wheel_current_pose;
  wheel_current_pose.header = msg->header;
  wheel_current_pose.pose = msg->pose;
  geometry_utils::Transform3 wheel_transform = GetTransform(wheel_last_pose_, wheel_current_pose);
  double x = wheel_transform.translation(0);
  double y = wheel_transform.translation(1);
  double z = wheel_transform.translation(2);
  wheel_delta_ = std::sqrt(x * x + y * y + z * z);
  wheel_last_pose_ = wheel_current_pose;
}

void LidarSlipDetection::LidarOdometryCallback(const Odometry::ConstPtr& msg) {
  // Callback function for lidar odometry
  double slip_amount = 0.0;
  bool slip_status = false;
  PoseCovStamped lidar_current_pose;
  lidar_current_pose.header = msg->header;
  lidar_current_pose.pose = msg->pose;
  geometry_utils::Transform3 lidar_transform= GetTransform(lidar_last_pose_, lidar_current_pose);
  double x = lidar_transform.translation(0);
  double y = lidar_transform.translation(1);
  double z = lidar_transform.translation(2);
  lidar_delta_ = std::sqrt(x * x + y * y + z * z);
  if (lidar_delta_ > 0) {
    slip_amount = abs(wheel_delta_ - lidar_delta_);
    ROS_INFO_STREAM("slip: " << slip_amount);
    if (slip_amount > slip_threshold_) {
      slip_amount = 1;
      slip_status = true;
    } else {
      slip_amount = 0;
    } 
  }
  PublishLidarSlipAmount(slip_amount, lidar_slip_amount_pub_);
  PublishLidarSlipStatus(slip_status, lidar_slip_status_pub_);
  lidar_last_pose_ = lidar_current_pose;
}

void LidarSlipDetection::ConditionNumberCallback(const std_msgs::Float64 &condition_number) {
  // Callback function for condition number

}

void LidarSlipDetection::InitializePose(PoseCovStamped pose) {
  pose.header.stamp = ros::Time::now();
  pose.pose.pose.position.x = 0;
  pose.pose.pose.position.y = 0;
  pose.pose.pose.position.z = 0;
  pose.pose.pose.orientation.x = 0;
  pose.pose.pose.orientation.y = 0;
  pose.pose.pose.orientation.z = 0;
  pose.pose.pose.orientation.w = 1;
}

geometry_utils::Transform3 LidarSlipDetection::GetTransform(
    const PoseCovStamped first_pose, const PoseCovStamped second_pose) {
  // Gets the delta between two pose stamped
  auto pose_first = gr::FromROS(first_pose.pose.pose);
  auto pose_second = gr::FromROS(second_pose.pose.pose);
  auto pose_delta = gu::PoseDelta(pose_first, pose_second);
  return pose_delta;
}

void LidarSlipDetection::PublishLidarSlipAmount(double& slip_amount, const ros::Publisher& pub) {
  // Convert slipage value value to ROS format and publish.
  std_msgs::Float64 lidar_slip_amount;
  lidar_slip_amount.data = slip_amount;
  pub.publish(lidar_slip_amount);
}

void LidarSlipDetection::PublishLidarSlipStatus(bool& slip_status, const ros::Publisher& pub) {
  // Convert condition number value to ROS format and publish.
  std_msgs::Bool lidar_slip_status;
  lidar_slip_status.data = slip_status;
  pub.publish(lidar_slip_status);
}

// void LidarSlipDetection::PublishAvgConditionNumber(double& k, const ros::Publisher& pub) {
//   // Convert condition number value to ROS format and publish.
//   std_msgs::Float64 avg_condition_number;
//   avg_condition_number.data = k;
//   pub.publish(avg_condition_number);
// }