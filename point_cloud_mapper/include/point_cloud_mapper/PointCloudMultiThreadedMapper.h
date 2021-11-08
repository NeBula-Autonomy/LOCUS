#pragma once

#include <ros/ros.h>

#include <pcl/octree/octree_search.h>
#include <pcl_ros/point_cloud.h>

#include <mutex>
#include <thread>

#include <core_msgs/MapInfo.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>
#include <pcl/filters/crop_box.h>

#include "IPointCloudMapper.h"

//#include <utils/CommonStructs.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt64.h>
#include <utils/PointCloudTypes.h>

class PointCloudMultiThreadedMapper : public IPointCloudMapper {
public:
  typedef pcl::octree::OctreePointCloudSearch<Point> Octree;
  typedef std::vector<PointCloud> PointCloudBuffer;

  PointCloudMultiThreadedMapper();
  ~PointCloudMultiThreadedMapper();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n) override;

  // Resets the octree and stored points to an empty map. This is used when
  // closing loops or otherwise regenerating the map from scratch.
  void Reset();

  // Adds a set of points to the octree. Only inserts points if one does not
  // already exist in the corresponding voxel. Output the points from the input
  // that ended up being inserted into the octree.
  // Effective C++ item 37
  bool InsertPoints(const PointCloud::ConstPtr& points,
                    PointCloud* incremental_points) override;

  // Returns the approximate nearest neighbor for every point in the input point
  // cloud. Localization to the map can be performed by doing ICP between the
  // input and output. Returns true if at least one of the inputs points had a
  // nearest neighbor.
  virtual bool ApproxNearestNeighbors(const PointCloud& points,
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

  // For multi-threaded map sliding window
  void RefreshThread();

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
  bool map_b_updated_;
  bool b_publish_only_with_subscribers_;

  // When a loop closure occurs, this flag enables a user to unsubscribe from
  // and resubscribe to the incremental map topic in order to re-draw the map.
  bool incremental_unsubscribed_;

  // Containers storing the map and its structure.
  PointCloud::Ptr map_data_;
  PointCloud::Ptr map_data_b_;
  Octree::Ptr map_octree_;
  Octree::Ptr map_octree_b_;

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
  std::thread refresh_thread_;
  mutable std::mutex map_mutex_;
  mutable std::mutex map_mutex_b_;
  mutable std::mutex map_frozen_mutex_;

  pcl::CropBox<Point> box_filter_;
  int box_filter_size_;

  std::string refresh_id_;

  PointCloud::Ptr history_;
  bool b_keep_history_;
  bool b_add_history_to_a_;
  bool b_add_history_to_b_;
};
