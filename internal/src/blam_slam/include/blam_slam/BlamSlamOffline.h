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
 * Author: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// This class can be used for running SLAM offline by loading messages from a
// ROS bagfile and processing them one-by-one. This allows for debugging without
// dealing with real-time issues, such as backed up message buffers or processor
// throttling. The offline processor by default will play back
//
///////////////////////////////////////////////////////////////////////////////

#ifndef BLAM_SLAM_OFFLINE_H
#define BLAM_SLAM_OFFLINE_H

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <parameter_utils/ParameterUtils.h>
#include <measurement_synchronizer/MeasurementSynchronizer.h>
#include <rosgraph_msgs/Clock.h>

#include "BlamSlam.h"

namespace pu = parameter_utils;

class BlamSlamOffline {
 public:
  BlamSlamOffline() {}
  ~BlamSlamOffline() {}

  bool Initialize(const ros::NodeHandle& n) {
    name_ = ros::names::append(n.getNamespace(), "BlamSlamOffline");

    if (!slam_.Initialize(n, true)) {
      ROS_ERROR("%s: Failed to initialize BLAM SLAM.", name_.c_str());
      return false;
    }

    if (!LoadParameters(n)) {
      ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
      return false;
    }

    if (!RegisterCallbacks(n)) {
      ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
      return false;
    }

    if (!LoadBagfile()) {
      ROS_ERROR("%s: Failed to load bag file.", name_.c_str());
      return false;
    }

    if (!ProcessBagfile()) {
      ROS_ERROR("%s: Failed to process bag file.", name_.c_str());
      return false;
    }

    return true;
  }

 private:

  bool RegisterCallbacks(const ros::NodeHandle& n) {
    ros::NodeHandle nl;
    const std::string ns = ros::this_node::getNamespace();

    scan_pub_ = nl.advertise<BlamSlam::PointCloud>(scan_topic_, 10, false);
    clock_pub_ = nl.advertise<rosgraph_msgs::Clock>("/clock", 10, false);

    return true;
  }

  bool LoadParameters(const ros::NodeHandle& n) {

    // Check that bag file exists.
    if (!pu::Get("filename/bag", bag_filename_)) return false;
    boost::filesystem::path bag_path(bag_filename_);
    if (!boost::filesystem::exists(bag_path)) {
      ROS_ERROR("%s: Bag file does not exist.", name_.c_str());
      return false;
    }

    // For time_end, -1.0 means "end of file". For time_scale, -1.0 means
    // "process messages as fast as possible".
    pu::Get("time_start", time_start_, 0.0);
    pu::Get("time_end", time_end_, -1.0);
    pu::Get("time_scale", time_scale_, -1.0);

    // Load bagfile topic names.
    if (!pu::Get("scan_topic", scan_topic_)) return false;

    return true;
  }

  bool LoadBagfile() {
    const std::string ns = ros::this_node::getNamespace();

    // Allow for more topics in the future.
    std::vector<std::string> topics;
    topics.push_back(scan_topic_);

    ROS_INFO("%s: Processing the following topics:", name_.c_str());
    for (unsigned int ii = 0; ii < topics.size(); ++ii)
      printf("\t%s\n", topics[ii].c_str());

    rosbag::Bag bag(bag_filename_, rosbag::bagmode::Read);
    rosbag::View preview(bag, rosbag::TopicQuery(topics),
                         ros::TIME_MIN, ros::TIME_MAX);
    ros::Time unix_start_time = preview.getBeginTime();
    ros::Time unix_end_time = preview.getEndTime();

    if (time_start_ > 0.0) {
      if (time_start_ < 684561600.0)
        unix_start_time += ros::Duration(time_start_);
      else
        unix_start_time = ros::Time() + ros::Duration(time_start_);
    }
    else
      unix_start_time = ros::TIME_MIN;

    if (time_end_ > -1.0) {
      if (time_end_ < 684561600.0)
        unix_end_time = preview.getBeginTime() + ros::Duration(time_end_);
      else
        unix_end_time = ros::Time() + ros::Duration(time_end_);
    }
    else
      unix_end_time = ros::TIME_MAX;

    ROS_INFO("%s: Bag start time    = %16.6f \t Bag end time    = %16.6f",
             name_.c_str(), preview.getBeginTime().toSec(),
             preview.getEndTime().toSec());
    ROS_INFO("%s: Replay start time = %16.6f \t Replay end time = %16.6f",
             name_.c_str(), unix_start_time.toSec(), unix_end_time.toSec());

    rosbag::View view(bag, rosbag::TopicQuery(topics),
                      unix_start_time, unix_end_time);

    BOOST_FOREACH(rosbag::MessageInstance const m, view) {
      if (!m.getTopic().compare(scan_topic_)) {
        BlamSlam::PointCloud::ConstPtr msg =
            m.instantiate<BlamSlam::PointCloud>();

        if (msg != NULL)
          synchronizer_.AddPCLPointCloudMessage(msg);
      } else  {
        ROS_ERROR("%s: Unknown topic in bagfile: %s.", name_.c_str(),
                  m.getTopic().c_str());
      }
    }

    bag.close();
    return true;
  }

  bool ProcessBagfile() {
    ROS_INFO("%s: Sorting messages.", name_.c_str());
    synchronizer_.SortMessages();

    ros::WallTime wall_time_start = ros::WallTime::now();
    ros::Time bag_time_start, bag_time;
    bool bag_time_start_set = false;

    ROS_INFO("%s: Processing messages.", name_.c_str());
    MeasurementSynchronizer::sensor_type type;
    unsigned int index = 0;

    while (synchronizer_.GetNextMessage(&type, &index)) {
      switch (type) {
        case MeasurementSynchronizer::PCL_POINTCLOUD: {
          const MeasurementSynchronizer::Message<
              BlamSlam::PointCloud>::ConstPtr& m =
              synchronizer_.GetPCLPointCloudMessage(index);

          slam_.ProcessPointCloudMessage(m->msg);
          scan_pub_.publish(m->msg);

          rosgraph_msgs::Clock clock_msg;
          clock_msg.clock = ros::Time().fromNSec(m->msg->header.stamp * 1e3);
          clock_pub_.publish(clock_msg);

          if (!bag_time_start_set) {
            bag_time_start = ros::Time().fromNSec(m->msg->header.stamp * 1e3);
            bag_time_start_set = true;
          }
          bag_time = ros::Time().fromNSec(m->msg->header.stamp * 1e3);

          break;
        }
        default: {
          ROS_WARN("%s: Unhandled measurement type: %s\n", name_.c_str(),
                   MeasurementSynchronizer::GetTypeString(type).c_str());
          break;
        }
      }

      // Check for kill signals.
      if (!ros::ok())
        break;

      ros::spinOnce();

      // Alter time scale.
      if (time_scale_ > 0.0) {
        ros::WallDuration dt_wall = ros::WallTime::now() - wall_time_start;
        ros::Duration dt_bag = bag_time - bag_time_start;
        double dt_sleep = dt_bag.toSec()/time_scale_ - dt_wall.toSec();
        if (dt_sleep > 0.0)
          ros::WallDuration(dt_sleep).sleep();
      }
    }

    synchronizer_.ClearMessages();

    const double total_wall_time =
        (ros::WallTime::now() - wall_time_start).toSec();
    const double total_bag_time = (bag_time - bag_time_start).toSec();
    ROS_INFO("%s: Finished processing bag file, %lf percent of real-time.",
             name_.c_str(), (total_bag_time / total_wall_time) * 100.f);
    return true;
  }

  std::string name_;
  std::string bag_filename_;

  std::string scan_topic_;

  double time_start_;
  double time_end_;
  double time_scale_;

  MeasurementSynchronizer synchronizer_;
  BlamSlam slam_;

  ros::Publisher scan_pub_;
  ros::Publisher clock_pub_;
};

#endif
