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

#ifndef MEASUREMENT_SYNCHRONIZER_H
#define MEASUREMENT_SYNCHRONIZER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include <memory>
#include <vector>

class MeasurementSynchronizer {
 public:
  MeasurementSynchronizer();
  ~MeasurementSynchronizer();

  // Enums for all valid sensor message types.
  typedef enum {
    POINTCLOUD,
    PCL_POINTCLOUD
  } sensor_type;

  // Basic sorting, querying, clearing.
  void SortMessages();
  bool GetNextMessage(sensor_type* type, unsigned int* index);
  bool NextMessageExists();
  void ClearMessages();

  // Templated message type for holding generic sensor messages.
  template<typename T>
  struct Message {
    typename T::ConstPtr msg;
    std::string tag;
    typedef std::shared_ptr<Message<T>> Ptr;
    typedef std::shared_ptr<const Message<T>> ConstPtr;
    Message(const typename T::ConstPtr m, const std::string& t)
        : msg(m), tag(t) {}
  };

  // Typedefs for queues of all sensor types.
  typedef std::vector<Message<sensor_msgs::PointCloud2>::ConstPtr> pcld_queue;
  typedef std::vector<Message<pcl::PointCloud<pcl::PointXYZ>>::ConstPtr>
      pcl_pcld_queue;

  // Methods for accessing entire queues of accumulated measurements.
  const pcld_queue& GetPointCloudMessages();
  const pcl_pcld_queue& GetPCLPointCloudMessages();

  // Methods for accessing a single measurement by index.
  const Message<sensor_msgs::PointCloud2>::ConstPtr& GetPointCloudMessage(
      unsigned int index);
  const Message<pcl::PointCloud<pcl::PointXYZ>>::ConstPtr&
      GetPCLPointCloudMessage(unsigned int index);

  // Methods for adding sensor measurements of specific types.
  void AddPointCloudMessage(const sensor_msgs::PointCloud2::ConstPtr& msg,
                            const std::string& tag = std::string());
  void AddPCLPointCloudMessage(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg,
      const std::string& tag = std::string());

  // Static enum to string conversion.
  static std::string GetTypeString(const sensor_type& type) {
    switch(type) {
      case POINTCLOUD:
        return std::string("POINTCLOUD");
      case PCL_POINTCLOUD:
        return std::string("PCL_POINTCLOUD");
      // No default to force compile-time error.
    }
  }

 private:
  // Templated message for sorting generic sensor messages by timestamp.
  struct TimestampedType {
    double time;
    sensor_type type;
    unsigned int index;
    TimestampedType(double t, sensor_type s, unsigned int i)
        : time(t), type(s), index(i) {}
    typedef std::shared_ptr<TimestampedType> Ptr;
    typedef std::shared_ptr<const TimestampedType> ConstPtr;
  };

  // Sorting function.
  static bool CompareTimestamps(const TimestampedType::ConstPtr& lhs,
                                const TimestampedType::ConstPtr& rhs) {
    return ((lhs->time < rhs->time) ||
            ((lhs->time == rhs->time) && (lhs->type < rhs->type)));
  }

  // Queues of sensor messages.
  pcld_queue pending_pclds_;
  pcl_pcld_queue pending_pcl_pclds_;

  unsigned int pending_index_;
  std::vector<TimestampedType::ConstPtr> sensor_ordering_;
};

#endif
