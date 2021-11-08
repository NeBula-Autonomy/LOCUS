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

#ifndef POINT_CLOUD_MAPPER_H
#define POINT_CLOUD_MAPPER_H

#include <ros/ros.h>

#include <pcl/octree/octree_search.h>
#include <pcl_ros/point_cloud.h>

#include <mutex>
#include <thread>

#include <core_msgs/MapInfo.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <pcl/filters/crop_box.h>

#include <std_msgs/Float64.h>
#include <std_msgs/UInt64.h>

#include "IPointCloudMapper.h"

//#include <utils/CommonStructs.h>
#include <utils/PointCloudTypes.h>
class PointCloudMapper : public IPointCloudMapper {
public:
  typedef pcl::octree::OctreePointCloudSearch<Point> Octree;

  PointCloudMapper();
  ~PointCloudMapper();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n) override;

  // Resets the octree and stored points to an empty map. This is used when
  // closing loops or otherwise regenerating the map from scratch.
  void Reset() override;

  // Adds a set of points to the octree. Only inserts points if one does not
  // already exist in the corresponding voxel. Output the points from the input
  // that ended up being inserted into the octree.
  bool InsertPoints(const PointCloud::ConstPtr& points,
                    PointCloud* incremental_points) override;

  // Returns the approximate nearest neighbor for every point in the input point
  // cloud. Localization to the map can be performed by doing ICP between the
  // input and output. Returns true if at least one of the inputs points had a
  // nearest neighbor.
  bool ApproxNearestNeighbors(const PointCloud& points,
                              PointCloud* neighbors) override;

  // Publish map for visualization. This can be expensive so it is not called
  // from inside, as opposed to PublishMapUpdate().
  void PublishMap();
  void PublishMapFrozen();

  // Publish map info for analysis
  void PublishMapInfo();

  // Map Sliding Window 2 ---------------------------------------------------
  void SetBoxFilterSize(const int box_filter_size) override;
  void Refresh(const geometry_utils::Transform3& current_pose) override;

private:
  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Threaded version to avoid blocking SLAM when the map gets big.
  void PublishMapThread();
  void PublishMapFrozenThread();

  // Publish map updates for visualization.
  void PublishMapUpdate(const PointCloud& incremental_points);

  // The node's name.
  std::string name_;

  // Frame id for publishing map.
  std::string fixed_frame_id_;

  // Boolean initialization flag that is set after success from LoadParameters.
  bool initialized_;

  // Boolean to only publish the map if it has been updated recently.
  bool map_updated_;
  bool b_publish_only_with_subscribers_;

  // When a loop closure occurs, this flag enables a user to unsubscribe from
  // and resubscribe to the incremental map topic in order to re-draw the map.
  bool incremental_unsubscribed_;

  // Containers storing the map and its structure.

  Octree::Ptr map_octree_;
  // Storage if we are using a rolling buffer

  // Map parameters.
  double octree_resolution_;

  // Map info pub paramters
  bool b_publish_map_info_;
  double volume_voxel_size;

  // Map publisher.
  ros::Publisher map_pub_;
  ros::Publisher map_frozen_pub_;
  ros::Publisher incremental_map_pub_;
  ros::Publisher map_info_pub_;

  ros::Publisher search_in_map_pub_;
  ros::Publisher delete_from_map_pub_;
  ros::Publisher adding_to_map_pub_;
  ros::Publisher no_insert_points_pub_;
  ros::Publisher no_nearest_points_pub_;

  std::thread publish_thread_;
  std::thread publish_frozen_thread_;
  mutable std::mutex map_mutex_;
  mutable std::mutex map_frozen_mutex_;

  // Map Sliding Window 2 -----------------
  pcl::CropBox<Point> box_filter_;
  int box_filter_size_;
};

#endif
