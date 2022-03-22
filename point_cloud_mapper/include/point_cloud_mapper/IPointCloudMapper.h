#pragma once

#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <mutex>
#include <thread>

#include <core_msgs/MapInfo.h>

#include <geometry_utils/GeometryUtilsROS.h>
#include <geometry_utils/Transform3.h>

//#include <utils/CommonStructs.h>
//#include <utils/PointCloudTypes.h>
#include <frontend_utils/CommonStructs.h>
class IPointCloudMapper
{
public:
  using Ptr = std::shared_ptr<IPointCloudMapper>;
  IPointCloudMapper(){};
  virtual ~IPointCloudMapper(){};

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  virtual bool Initialize(const ros::NodeHandle& n) = 0;

  // Resets the octree and stored points to an empty map. This is used when
  // closing loops or otherwise regenerating the map from scratch.
  virtual void Reset() = 0;

  // Adds a set of points to the datastructure
  virtual bool InsertPoints(const PointCloudF::ConstPtr& points, PointCloudF* incremental_points) = 0;

  virtual bool ApproxNearestNeighbors(const PointCloudF& points, PointCloudF* neighbors) = 0;
  // Setting size of the local window map.
  virtual void SetBoxFilterSize(const int box_filter_size) = 0;

  // Publish map for visualization. This can be expensive so it is not called
  // from inside, as opposed to PublishMapUpdate().
  virtual void PublishMap() = 0;
  virtual void PublishMapFrozen() = 0;
  // Publish map info for analysis
  virtual void PublishMapInfo() = 0;

  // Getter for the point cloud
  PointCloudF::Ptr GetMapData()
  {
    return map_data_;
  }

  void SetupNumberThreads(int no_threads)
  {
    number_threads_ = no_threads;
    ROS_INFO_STREAM("Setting up number threads for local mapping: " << number_threads_);
  }

  virtual void Refresh(const geometry_utils::Transform3& current_pose) = 0;

  void UpdateCurrentPose(const geometry_utils::Transform3& current_pose)
  {
    current_pose_estimate_ = current_pose;
  }

protected:
  PointCloudF::Ptr map_data_;
  geometry_utils::Transform3 current_pose_estimate_;
  int number_threads_{ 1 };
};
