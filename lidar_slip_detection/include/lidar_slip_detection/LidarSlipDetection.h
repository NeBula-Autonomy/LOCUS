/*
 * Copyright (c) 2019, NASA Jet Propulsion Laboratory - California Institute
 * of Technology. * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following conditions
 * are met:
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

#ifndef LIDAR_SLIP_DETECTION_H
#define LIDAR_SLIP_DETECTION_H

#include <ros/ros.h>
#include <ros/time.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <parameter_utils/ParameterUtils.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>

#include <eigen_conversions/eigen_msg.h>

#include <numeric>

namespace gu = geometry_utils;
namespace gr = geometry_utils::ros;

typedef nav_msgs::Odometry Odometry;
typedef geometry_msgs::PoseWithCovarianceStamped PoseCovStamped;

class LidarSlipDetection {
 public:
  LidarSlipDetection();
  ~LidarSlipDetection();

  bool Initialize(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  // Odometry Callbacks
  void WheelOdometryCallback(const Odometry::ConstPtr& msg);
  void LidarOdometryCallback(const Odometry::ConstPtr& msg);

  // Condition Number Callback
  void ConditionNumberCallback(
      const std_msgs::Float64::ConstPtr& condition_number);

  // Observability Vector Callback
  void ObservabilityVectorCallback(const geometry_msgs::Vector3::ConstPtr& msg);

  double wheel_delta_;
  double lidar_delta_;
  PoseCovStamped lidar_last_pose_;
  PoseCovStamped wheel_last_pose_;

 protected:
  // Subscribers
  ros::Subscriber condition_number_sub_;
  ros::Subscriber lidar_odom_sub_;
  ros::Subscriber wheel_odom_sub_;
  ros::Subscriber observability_sub_;

  // Publishers
  ros::Publisher slip_detection_from_odom_;
  ros::Publisher lidar_slip_status_pub_;
  ros::Publisher slip_detection_from_cov_;

  void InitializePose(PoseCovStamped first_pose);
  geometry_utils::Transform3 GetTransform(const PoseCovStamped first_pose,
                                          const PoseCovStamped second_pose);
  void PublishLidarSlipAmount(const double& slip_detection_odom,
                              const ros::Publisher& pub);
  void PublishLidarSlipStatus(const bool& slip_status,
                              const ros::Publisher& pub);
  bool LoadParameters(const ros::NodeHandle& n);
  void PublishConditionNumber(const double& slip_detection_cov,
                              const ros::Publisher& pub);

 private:
  std::string name_;
  double slip_amount_from_odom_;
  double slip_threshold_;
  double max_power_;
  double filter_size_;
  double observability_;
  double condition_number_;
  double observability_threshold_;
  std::vector<PoseCovStamped> last_lo_poses_;
  std::vector<PoseCovStamped> last_wo_poses_;

  bool b_use_wio_check_;
  bool b_use_condition_number_check_;
  bool b_use_observability_check_;
};

#endif
