#include <omp.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/search/impl/search.hpp>
#include <point_cloud_mapper/PointCloudIkdTreeMapper.h>
#include <std_msgs/String.h>

namespace pu = parameter_utils;

PointCloudIkdTreeMapper::PointCloudIkdTreeMapper()
  : initialized_(false), map_updated_(false), incremental_unsubscribed_(false) {
  map_data_.reset(new PointCloud);
}

PointCloudIkdTreeMapper::~PointCloudIkdTreeMapper() {
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

bool PointCloudIkdTreeMapper::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudIkdTreeMapper");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool PointCloudIkdTreeMapper::LoadParameters(const ros::NodeHandle& n) {
  // Load fixed frame.
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;

  // Load map parameters.
  if (!pu::Get("map/octree_resolution", octree_resolution_))
    return false;
  if (!pu::Get("map/b_publish_only_with_subscribers",
               b_publish_only_with_subscribers_))
    return false;

  ikdtree.set_downsample_param(octree_resolution_);

  initialized_ = true;

  return true;
}

bool PointCloudIkdTreeMapper::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  map_pub_ = nl.advertise<PointCloud>("octree_map", 10, true);
  incremental_map_pub_ =
      nl.advertise<PointCloud>("octree_map_updates", 10, true);
  map_frozen_pub_ = nl.advertise<PointCloud>("octree_map_frozen", 10, false);
  map_info_pub_ = nl.advertise<core_msgs::MapInfo>("map_info", 10, false);

  boxes_vis_pub_ = nl.advertise<visualization_msgs::MarkerArray>(
      "points_inside_boxes_to_remove", 10, true);

  search_ikdtree_pub_ =
      nl.advertise<std_msgs::Float64>("search_in_map", 10, false);
  delete_ikdtree_pub_ =
      nl.advertise<std_msgs::Float64>("delete_from_map", 10, false);
  adding_ikdtree_pub_ =
      nl.advertise<std_msgs::Float64>("adding_to_map", 10, false);

  insert_points_init_pub_ =
      nl.advertise<std_msgs::UInt64>("no_insert_points_pub", 10, false);

  nearest_points_init_pub_ =
      nl.advertise<std_msgs::UInt64>("no_nearest_points", 10, false);

  return true;
}

void PointCloudIkdTreeMapper::Reset() {
  // TODO: do we need this for local mapper?
  initialized_ = true;
}

visualization_msgs::Marker PointCloudIkdTreeMapper::CreateBoxToVisualize(
    const Eigen::Vector3f& current_position, int id) const {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.ns = "";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = current_position[0];
  marker.pose.position.y = current_position[1];
  marker.pose.position.z = current_position[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 2 * box_filter_size_;
  marker.scale.y = 2 * box_filter_size_;
  marker.scale.z = 2 * box_filter_size_;
  marker.color.a = 0.2;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  return marker;
}

void PointCloudIkdTreeMapper::UpdateBoxesLocationWithRespectToTheRobot(
    const geometry_utils::Transform3& current_pose) {
  for (size_t i = 0; i < box_filter_centers_.size(); ++i) {
    box_filters_[i].vertex_min[0] =
        static_cast<float>(current_pose.translation.data[0]) +
        box_filter_centers_[i][0] - box_filter_size_;
    box_filters_[i].vertex_min[1] =
        static_cast<float>(current_pose.translation.data[1]) +
        box_filter_centers_[i][1] - box_filter_size_;
    box_filters_[i].vertex_min[2] =
        static_cast<float>(current_pose.translation.data[2]) +
        box_filter_centers_[i][2] - box_filter_size_;
    box_filters_[i].vertex_max[0] =
        static_cast<float>(current_pose.translation.data[0]) +
        box_filter_centers_[i][0] + box_filter_size_;
    box_filters_[i].vertex_max[1] =
        static_cast<float>(current_pose.translation.data[1]) +
        box_filter_centers_[i][1] + box_filter_size_;
    box_filters_[i].vertex_max[2] =
        static_cast<float>(current_pose.translation.data[2]) +
        box_filter_centers_[i][2] + box_filter_size_;
  }

  if (boxes_vis_pub_.getNumSubscribers() != 0) {
    visualization_msgs::MarkerArray boxes_vis;
    for (int i = 0; i < static_cast<int>(box_filter_centers_.size()); ++i) {
      auto box_vis =
          CreateBoxToVisualize(current_pose.translation.Eigen().cast<float>() +
                                   box_filter_centers_[i],
                               i);
      boxes_vis.markers.push_back(box_vis);
    }
    boxes_vis_pub_.publish(boxes_vis);
  }
}

bool PointCloudIkdTreeMapper::InsertPoints(const PointCloud::ConstPtr& points,
                                           PointCloud* incremental_points) {
  if (!initialized_) {
    ROS_ERROR("%s: Not initialized.", name_.c_str());
    return false;
  }

  if (incremental_points == NULL) {
    ROS_ERROR("%s: Incremental point cloud argument is null.", name_.c_str());
    return false;
  }
  incremental_points->clear();

  auto no_points = std_msgs::UInt64();
  no_points.data = points->size();
  insert_points_init_pub_.publish(no_points);
  // Try to get the map mutex from the publisher. If the publisher is usingit,
  // we will just not insert this point cloud right now. It'll be added when the
  // map is regenerated by loop closure.

  if (map_mutex_.try_lock()) {
    if (ikdtree.size() == 0) {
      auto t_start = std::chrono::high_resolution_clock::now();

      ikdtree.Build(*points);
      auto t_end = std::chrono::high_resolution_clock::now();
      double elapsed_time_ms =
          std::chrono::duration<double, std::milli>(t_end - t_start).count();
      ROS_INFO_STREAM("Building init ikdtree: " << elapsed_time_ms);
    } else {
      PointVector pc_append(*points);
      auto t_start = std::chrono::high_resolution_clock::now();
      ikdtree.Add_Points(pc_append, false);
      auto t_end = std::chrono::high_resolution_clock::now();
      double elapsed_time_ms =
          std::chrono::duration<double, std::milli>(t_end - t_start).count();
      std_msgs::Float64 elapsed_time_ms_ros;
      elapsed_time_ms_ros.data = elapsed_time_ms;
      adding_ikdtree_pub_.publish(elapsed_time_ms_ros);
    }

    map_mutex_.unlock();
  } else {
    // This won't happen often.
    ROS_WARN(
        "%s: Failed to update map: map publisher has a hold of the thread. "
        "Turn off any subscriptions to the 3D map topic to prevent this from "
        "happening.",
        name_.c_str());
  }

  // Publish the incremental map update.
  //  incremental_points->header = points->header;
  //  incremental_points->header.frame_id = fixed_frame_id_;
  //  PublishMapUpdate(*incremental_points);

  map_updated_ = true;

  return true;
}

bool PointCloudIkdTreeMapper::ApproxNearestNeighbors(const PointCloud& points,
                                                     PointCloud* neighbors) {
  if (!initialized_) {
    ROS_ERROR("%s: Not initialized.", name_.c_str());
  }

  if (neighbors == NULL) {
    ROS_ERROR("%s: Output argument is null.", name_.c_str());
  }

  neighbors->points.clear();
  neighbors->resize(points.points.size());
  PointVector search_result;
  // Iterate over points in the input point cloud, finding the nearest
  // neighbor for every point and storing it in the output array.
  std_msgs::Float64 elapsed_time_ms_ros;
  auto no_points = std_msgs::UInt64();
  no_points.data = points.size();
  nearest_points_init_pub_.publish(no_points);
  auto t_start = std::chrono::high_resolution_clock::now();
  omp_set_num_threads(number_threads_);
#pragma omp parallel for schedule(dynamic, 1)
  for (size_t ii = 0; ii < points.points.size(); ++ii) {
    PointVector search_result;
    vector<float> PointDist;
    ikdtree.Nearest_Search(points[ii], 1, search_result, PointDist);

    neighbors->points[ii] = search_result[0];
  }

  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count();
  elapsed_time_ms_ros.data = elapsed_time_ms;
  search_ikdtree_pub_.publish(elapsed_time_ms_ros);

  return neighbors->points.size() > 0;
}

void PointCloudIkdTreeMapper::PublishMap() {
  if (map_pub_.getNumSubscribers() > 0 || !b_publish_only_with_subscribers_) {
    if (initialized_ && map_updated_) {
      // Use a new thread to publish the map to avoid blocking main thread
      // on concurrent calls.
      if (publish_thread_.joinable()) {
        publish_thread_.join();
      }
      publish_thread_ =
          std::thread(&PointCloudIkdTreeMapper::PublishMapThread, this);
    }
  }
}

void PointCloudIkdTreeMapper::PublishMapThread() {
  map_mutex_.lock();
  PointVector storage;
  storage.header.frame_id = fixed_frame_id_;
  ikdtree.flatten(ikdtree.Root_Node, storage, NOT_RECORD);
  map_pub_.publish(storage);
  // Don't publish again until we get another map update.
  map_updated_ = false;
  map_mutex_.unlock();
}

void PointCloudIkdTreeMapper::PublishMapFrozen() {
  if (initialized_ && map_frozen_pub_.getNumSubscribers() > 0) {
    // Use a new thread to publish the map to avoid blocking main thread
    // on concurrent calls.
    if (publish_frozen_thread_.joinable()) {
      publish_frozen_thread_.join();
    }
    publish_frozen_thread_ =
        std::thread(&PointCloudIkdTreeMapper::PublishMapFrozenThread, this);
  }
}

void PointCloudIkdTreeMapper::PublishMapFrozenThread() {
  map_frozen_mutex_.lock();
  ROS_INFO_STREAM("Publishing frozen map");
  map_frozen_pub_.publish(ikdtree.PCL_Storage);

  // Don't publish again until we get another map update.
  map_frozen_mutex_.unlock();
}

void PointCloudIkdTreeMapper::PublishMapUpdate(
    const PointCloud& incremental_points) {
  // Publish the incremental points for visualization.
  incremental_map_pub_.publish(incremental_points);
}

void PointCloudIkdTreeMapper::PublishMapInfo() {
  // todo: do we need this?
}

// Map Sliding Window 2 -----------------------------------------------

void PointCloudIkdTreeMapper::SetBoxFilterSize(const int box_filter_size) {
  box_filter_size_ = static_cast<float>(box_filter_size);
  ROS_INFO_STREAM(
      "Setting up box filter with half length: " << box_filter_size);

  box_filters_.push_back(BoxPointType(
      {Eigen::Vector3f{-box_filter_size_, -box_filter_size_, -box_filter_size_},
       Eigen::Vector3f{box_filter_size_, box_filter_size_, box_filter_size_}}));

  box_filters_.push_back(BoxPointType(
      {Eigen::Vector3f{-box_filter_size_, -box_filter_size_, -box_filter_size_},
       Eigen::Vector3f{box_filter_size_, box_filter_size_, box_filter_size_}}));

  box_filters_.push_back(BoxPointType(
      {Eigen::Vector3f{-box_filter_size_, -box_filter_size_, -box_filter_size_},
       Eigen::Vector3f{box_filter_size_, box_filter_size_, box_filter_size_}}));

  box_filters_.push_back(BoxPointType(
      {Eigen::Vector3f{-box_filter_size_, -box_filter_size_, -box_filter_size_},
       Eigen::Vector3f{box_filter_size_, box_filter_size_, box_filter_size_}}));

  box_filter_centers_.push_back(
      Eigen::Vector3f(0, 2.0f * box_filter_size_, 0.0f));
  box_filter_centers_.push_back(
      Eigen::Vector3f(0, -2.0f * box_filter_size_, 0.0f));
  box_filter_centers_.push_back(
      Eigen::Vector3f(-2.0f * box_filter_size_, 0.0f, 0.0f));
  box_filter_centers_.push_back(
      Eigen::Vector3f(2.0f * box_filter_size_, 0.0f, 0.0f));
  // maybe to remove at some point since if the local map is big there is no
  // chance that we get measurements there from lidar sensor
  box_filter_centers_.push_back(
      Eigen::Vector3f(2.0f * box_filter_size_, 2.0f * box_filter_size_, 0.0f));
  box_filter_centers_.push_back(Eigen::Vector3f(
      -2.0f * box_filter_size_, -2.0f * box_filter_size_, 0.0f));
  box_filter_centers_.push_back(
      Eigen::Vector3f(-2.0f * box_filter_size_, 2.0f * box_filter_size_, 0.0f));
  box_filter_centers_.push_back(
      Eigen::Vector3f(2.0f * box_filter_size_, -2.0f * box_filter_size_, 0.0f));

  box_filters_.push_back(BoxPointType(
      {Eigen::Vector3f{-box_filter_size_, -box_filter_size_, -box_filter_size_},
       Eigen::Vector3f{box_filter_size_, box_filter_size_, box_filter_size_}}));

  box_filters_.push_back(BoxPointType(
      {Eigen::Vector3f{-box_filter_size_, -box_filter_size_, -box_filter_size_},
       Eigen::Vector3f{box_filter_size_, box_filter_size_, box_filter_size_}}));

  box_filters_.push_back(BoxPointType(
      {Eigen::Vector3f{-box_filter_size_, -box_filter_size_, -box_filter_size_},
       Eigen::Vector3f{box_filter_size_, box_filter_size_, box_filter_size_}}));

  box_filters_.push_back(BoxPointType(
      {Eigen::Vector3f{-box_filter_size_, -box_filter_size_, -box_filter_size_},
       Eigen::Vector3f{box_filter_size_, box_filter_size_, box_filter_size_}}));
}

void PointCloudIkdTreeMapper::Refresh(
    const geometry_utils::Transform3& current_pose) {
  // ROS_WARN_STREAM(
  //     "##################################################################");
  // ROS_WARN_STREAM("If you see this, something is wrong!!! In
  // lo_settings.yaml
  // "
  //                 "b_enable_msw should be false.");
  // ROS_WARN_STREAM(
  //     "##################################################################");

  UpdateBoxesLocationWithRespectToTheRobot(current_pose_estimate_);
  if (map_mutex_.try_lock()) {
    auto t_start = std::chrono::high_resolution_clock::now();
    ikdtree.Delete_Point_Boxes(box_filters_);
    map_mutex_.unlock();
    auto t_end = std::chrono::high_resolution_clock::now();
    auto elapsed_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();
    std_msgs::Float64 elapsed_time_ms_ros;
    elapsed_time_ms_ros.data = elapsed_time_ms;
    delete_ikdtree_pub_.publish(elapsed_time_ms_ros);
  }
}
