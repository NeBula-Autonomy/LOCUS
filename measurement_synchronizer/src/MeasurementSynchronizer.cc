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

#include <measurement_synchronizer/MeasurementSynchronizer.h>

MeasurementSynchronizer::MeasurementSynchronizer() : pending_index_(0) {}
MeasurementSynchronizer::~MeasurementSynchronizer() {}

void MeasurementSynchronizer::SortMessages() {
  sensor_ordering_.clear();

  // Accumulate all new messages in a single list.
  unsigned int ii = 0;
  for (pcld_queue::const_iterator it = pending_pclds_.begin();
       it != pending_pclds_.end(); ++it, ++ii) {
    TimestampedType::Ptr p = TimestampedType::Ptr(
        new TimestampedType((*it)->msg->header.stamp.toSec(), POINTCLOUD, ii));
    sensor_ordering_.push_back(p);
  }

  ii = 0;
  for (pcl_pcld_queue::const_iterator it = pending_pcl_pclds_.begin();
       it != pending_pcl_pclds_.end(); ++it, ++ii) {
    ros::Time stamp;
    stamp.fromNSec((*it)->msg->header.stamp*1e3);
    TimestampedType::Ptr p = TimestampedType::Ptr(
        new TimestampedType(stamp.toSec(), PCL_POINTCLOUD, ii));
    sensor_ordering_.push_back(p);
  }

  ii = 0;
  for (imu_queue::const_iterator it = pending_imus_.begin();
       it != pending_imus_.end(); ++it, ++ii) {
    TimestampedType::Ptr p = TimestampedType::Ptr(
        new TimestampedType((*it)->msg->header.stamp.toSec(), IMU, ii));
    sensor_ordering_.push_back(p);
  }  

  ii = 0;
  for (odom_queue::const_iterator it = pending_odoms_.begin();
       it != pending_odoms_.end(); ++it, ++ii) {
    TimestampedType::Ptr p = TimestampedType::Ptr(
        new TimestampedType((*it)->msg->header.stamp.toSec(), ODOM, ii));
    sensor_ordering_.push_back(p);
  }  

  ii = 0;
  for (gt_queue::const_iterator it = pending_gts_.begin();
       it != pending_gts_.end(); ++it, ++ii) {
    TimestampedType::Ptr p = TimestampedType::Ptr(
        new TimestampedType((*it)->msg->header.stamp.toSec(), GT, ii));
    sensor_ordering_.push_back(p);
  }  

  // Sort the list by time.
  std::sort(sensor_ordering_.begin(), sensor_ordering_.end(),
            MeasurementSynchronizer::CompareTimestamps);

  pending_index_ = 0;
}

bool MeasurementSynchronizer::GetNextMessage(sensor_type* type,
                                             unsigned int* index) {
  if (type == NULL || index == NULL) {
    ROS_WARN("[MeasurementSynchronizer.cc]: Type or index is null.");
    return false;
  }

  if (pending_index_ >= sensor_ordering_.size()) {
    return false;
  }

  *type = sensor_ordering_[pending_index_]->type;
  *index = sensor_ordering_[pending_index_]->index;

  pending_index_++;
  return true;
}

bool MeasurementSynchronizer::NextMessageExists() {
  return pending_index_ < sensor_ordering_.size();
}

void MeasurementSynchronizer::ClearMessages() {
  pending_pclds_.clear();
  pending_pcl_pclds_.clear();
  pending_imus_.clear(); 
  pending_odoms_.clear(); 
  pending_gts_.clear(); 
}

const MeasurementSynchronizer::pcld_queue&
MeasurementSynchronizer::GetPointCloudMessages() {
  return pending_pclds_;
}

const MeasurementSynchronizer::pcl_pcld_queue&
MeasurementSynchronizer::GetPCLPointCloudMessages() {
  return pending_pcl_pclds_;
}

const MeasurementSynchronizer::imu_queue&
MeasurementSynchronizer::GetImuMessages() {
  return pending_imus_;
}

const MeasurementSynchronizer::odom_queue&
MeasurementSynchronizer::GetOdomMessages() {
  return pending_odoms_;
}

const MeasurementSynchronizer::gt_queue&
MeasurementSynchronizer::GetGtMessages() {
  return pending_gts_;
}

const MeasurementSynchronizer::Message<sensor_msgs::PointCloud2>::ConstPtr&
MeasurementSynchronizer::GetPointCloudMessage(unsigned int index) {
  return pending_pclds_[index];
}

const MeasurementSynchronizer::Message<
    pcl::PointCloud<pcl::PointXYZ>>::ConstPtr&
MeasurementSynchronizer::GetPCLPointCloudMessage(unsigned int index) {
  return pending_pcl_pclds_[index];
}

const MeasurementSynchronizer::Message<sensor_msgs::Imu>::ConstPtr&
MeasurementSynchronizer::GetImuMessage(unsigned int index) {
  return pending_imus_[index];
}

const MeasurementSynchronizer::Message<nav_msgs::Odometry>::ConstPtr&
MeasurementSynchronizer::GetOdomMessage(unsigned int index) {
  return pending_odoms_[index];
}

const MeasurementSynchronizer::Message<geometry_msgs::PoseStamped>::ConstPtr&
MeasurementSynchronizer::GetGtMessage(unsigned int index) {
  return pending_gts_[index];
}

void MeasurementSynchronizer::AddPointCloudMessage(
    const sensor_msgs::PointCloud2::ConstPtr& msg, const std::string& tag) {
  Message<sensor_msgs::PointCloud2>::Ptr p(
      new Message<sensor_msgs::PointCloud2>(msg, tag));
  pending_pclds_.push_back(p);
}

void MeasurementSynchronizer::AddPCLPointCloudMessage(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg,
    const std::string& tag) {
  Message<pcl::PointCloud<pcl::PointXYZ>>::Ptr p(
      new Message<pcl::PointCloud<pcl::PointXYZ>>(msg, tag));
  pending_pcl_pclds_.push_back(p);
}

// TODO: Implementing this without a tag, is it needed? ---> It is needed in the stage of Message creation as in MeasurementSynchronizer.h 
void MeasurementSynchronizer::AddImuMessage(const sensor_msgs::Imu::ConstPtr& msg){
    std::string imu_tag = "imu_tag";
    Message<sensor_msgs::Imu>::Ptr p( new Message<sensor_msgs::Imu>(msg, imu_tag));
    pending_imus_.push_back(p); 
}

void MeasurementSynchronizer::AddOdomMessage(const nav_msgs::Odometry::ConstPtr& msg){
    std::string odom_tag = "odom_tag";
    Message<nav_msgs::Odometry>::Ptr p( new Message<nav_msgs::Odometry>(msg, odom_tag));
    pending_odoms_.push_back(p); 
}

void MeasurementSynchronizer::AddGtMessage(const geometry_msgs::PoseStamped::ConstPtr& msg){
    std::string gt_tag = "gt_tag"; 
    Message<geometry_msgs::PoseStamped>::Ptr p( new Message<geometry_msgs::PoseStamped>(msg, gt_tag));
    pending_gts_.push_back(p); 
}