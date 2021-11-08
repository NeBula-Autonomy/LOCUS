#pragma once

#include "IPointCloudMapper.h"
//#include <utils/CommonStructs.h>
#include <utils/PointCloudTypes.h>
class SimplePointCloudMapper : public IPointCloudMapper {
public:
  SimplePointCloudMapper();
  ~SimplePointCloudMapper();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n) override;

  // Resets stored points to an empty map. This is used when
  // closing loops or otherwise regenerating the map from scratch.
  void Reset() override;

  // Adds a set of points.
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
  void PublishMap() override;
  void PublishMapFrozen() override;
  // Publish map info for analysis
  void PublishMapInfo() override;

  void Refresh(const geometry_utils::Transform3& current_pose) override;
  void SetBoxFilterSize(const int box_filter_size) override;

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

  // Map info pub paramters
  bool b_publish_map_info_;
  double volume_voxel_size;

  // Map publisher.
  ros::Publisher map_pub_;
  ros::Publisher map_frozen_pub_;

  ros::Publisher map_info_pub_;
  std::thread publish_thread_;
  std::thread publish_frozen_thread_;
  mutable std::mutex map_mutex_;
  mutable std::mutex map_frozen_mutex_;

  int box_filter_size_;
};
