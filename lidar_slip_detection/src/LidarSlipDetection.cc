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

// #include <lidar_slip_detection/LidarSlipDetection.h>
#include <LidarSlipDetection.h>
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
  CreatePublishers(n);
  return true;
}

bool LidarSlipDetection::RegisterCallbacks(const ros::NodeHandle& n) {
  ROS_INFO("%s: Registering online callbacks in LidarSlipDetection");
  ros::NodeHandle nl(n);

  lidar_odom_sub_ = nl.subscribe(
      "lio_odom", 10, &LidarSlipDetection::LidarOdometryCallback, this);
  wheel_odom_sub_ = nl.subscribe(
      "wio_odom", 10, &LidarSlipDetection::WheelOdometryCallback, this);
  
  return true;
}

bool LidarSlipDetection::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  lidar_slip_amount_pub_ = nl.advertise<std_msgs::Float64>("lidar_slip_amount", 10, false);
  lidar_slip_status_pub_ = nl.advertise<std_msgs::Bool>("lidar_slip_status", 10, false);

  return true;
}


void LidarSlipDetection::LidarOdometryCallback(const Odometry::ConstPtr& msg) {
  // Callback function for lidar odometry
  PoseCovStamped first_pose, second_pose;
  InitializeInitialPose(first_pose);
}

void LidarSlipDetection::WheelOdometryCallback(const Odometry::ConstPtr& msg) {
  // Callback function for wheel odometry

}

void LidarSlipDetection::ConditionNumberCallback(const double &condition_number) {
  // Callback function for condition number

}

void LidarSlipDetection::InitializeInitialPose(PoseCovStamped first_pose) {
  first_pose.pose.pose.position.x = 0;
  first_pose.pose.pose.position.y = 0;
  first_pose.pose.pose.position.z = 0;
  first_pose.pose.pose.orientation.x = 0;
  first_pose.pose.pose.orientation.y = 0;
  first_pose.pose.pose.orientation.z = 0;
  first_pose.pose.pose.orientation.w = 1;
}

geometry_utils::Transform3 GetTransform(
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

// PublishLidarSlipAmount(lidar_slip_amount, lidar_slip_amount_pub_);
// PublishLidarSlipStatus(lidar_slip_status, lidar_slip_status_pub_);
