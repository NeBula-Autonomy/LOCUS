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

#include <visualization_msgs/MarkerArray.h>

#include "IPointCloudMapper.h"
#include "ikd_tree/ikd_Tree.h"

//#include <utils/CommonStructs.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt64.h>
#include <utils/PointCloudTypes.h>
class PointCloudIkdTreeMapper : public IPointCloudMapper {
public:
  // typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

  PointCloudIkdTreeMapper();
  ~PointCloudIkdTreeMapper();

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

  // Threaded version to avoid blocking SLAM when the map gets big.
  void PublishMapThread();
  void PublishMapFrozenThread();

  // Publish map updates for visualization.
  void PublishMapUpdate(const PointCloud& incremental_points);
  void UpdateBoxesLocationWithRespectToTheRobot(
      const geometry_utils::Transform3& current_pose);
  visualization_msgs::Marker
  CreateBoxToVisualize(const Eigen::Vector3f& current_position, int id) const;

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

  // Map parameters.
  double octree_resolution_;

  // Map publisher.
  ros::Publisher map_pub_;
  ros::Publisher map_frozen_pub_;
  ros::Publisher incremental_map_pub_;
  ros::Publisher map_info_pub_;
  ros::Publisher boxes_vis_pub_;
  ros::Publisher search_ikdtree_pub_;
  ros::Publisher delete_ikdtree_pub_;
  ros::Publisher adding_ikdtree_pub_;
  ros::Publisher insert_points_init_pub_;
  ros::Publisher nearest_points_init_pub_;

  std::thread publish_thread_;
  std::thread publish_frozen_thread_;
  mutable std::mutex map_mutex_;
  mutable std::mutex map_frozen_mutex_;

  std::vector<Eigen::Vector3f> box_filter_centers_;
  std::vector<BoxPointType> box_filters_;
  float box_filter_size_{25.0f};
  KD_TREE ikdtree;
  double filter_size_map_min{0.001};
};
