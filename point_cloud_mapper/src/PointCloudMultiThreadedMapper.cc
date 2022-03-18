#include <omp.h>
#include <std_msgs/String.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/octree/octree_iterator.h>
#include <parameter_utils/ParameterUtils.h>
#include <point_cloud_mapper/PointCloudMultiThreadedMapper.h>



namespace pu = parameter_utils;



PointCloudMultiThreadedMapper::PointCloudMultiThreadedMapper()
  : initialized_(false),
    map_updated_(false),
    incremental_unsubscribed_(false),
    b_inserted_points_(false),
    refresh_id_("a") {
  history_.reset(new PointCloud);
  map_data_.reset(new PointCloud);
  map_data_b_.reset(new PointCloud);
}



PointCloudMultiThreadedMapper::~PointCloudMultiThreadedMapper() {
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}



bool PointCloudMultiThreadedMapper::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudMultiThreadedMapper");
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  refresh_timer_ = n.createTimer(1, &PointCloudMultiThreadedMapper::RefreshTimerCallback, this);
  return true;
}



bool PointCloudMultiThreadedMapper::LoadParameters(const ros::NodeHandle& n) {
  if (!pu::Get("frame_id/fixed", fixed_frame_id_))
    return false;
  if (!pu::Get("map/octree_resolution", octree_resolution_))
    return false;
  if (!pu::Get("map/b_publish_only_with_subscribers", b_publish_only_with_subscribers_))
    return false;
  if (!pu::Get("map/b_publish_map_info", b_publish_map_info_))
    return false;
  if (!pu::Get("map/volume_voxel_size", volume_voxel_size))
    return false;
  map_data_->header.frame_id = fixed_frame_id_;
  map_data_b_->header.frame_id = fixed_frame_id_;
  map_octree_.reset(new Octree(octree_resolution_));
  map_octree_b_.reset(new Octree(octree_resolution_)); 
  map_octree_->setInputCloud(map_data_);
  initialized_ = true;
  return true;
}



bool PointCloudMultiThreadedMapper::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  map_pub_ = nl.advertise<PointCloud>("octree_map", 10, true);
  return true;
}



void PointCloudMultiThreadedMapper::SetBoxFilterSize(const int box_filter_size) {
  box_filter_size_ = box_filter_size;
  box_filter_.setMin(Eigen::Vector4f(-box_filter_size_, -box_filter_size_, -box_filter_size_, 1.0));
  box_filter_.setMax(Eigen::Vector4f(box_filter_size_, box_filter_size_, box_filter_size_, 1.0));
}



bool PointCloudMultiThreadedMapper::ApproxNearestNeighbors(const PointCloud& points, PointCloud* neighbors) {
  if (!initialized_) {
    ROS_ERROR("%s: Not initialized.", name_.c_str());
  }
  if (neighbors == NULL) {
    ROS_ERROR("%s: Output argument is null.", name_.c_str());
  }
  if (!b_inserted_points_) {
    ROS_WARN("Can't retrieve neighbors, returning false");
    return false;
  }
  
  neighbors->clear(); 
  neighbors->resize(points.points.size());

  std::string refresh_id;
  {
    std::lock_guard<std::mutex> lock(refresh_id_mutex_);
    refresh_id = refresh_id_;
  }      

  // Iterate over points in the input point cloud, finding the nearest neighbor for every point and storing it in the output array
  int enable_omp = (1 < 4); 
  omp_set_num_threads(4);
  #pragma omp parallel for schedule(dynamic, 1)
  for (size_t i = 0; i < points.points.size(); ++i) {
    float unused = 0.f;
    int result_index = -1;
    if (refresh_id == "a") {
      map_octree_->approxNearestSearch(points.points[i], result_index, unused);
      if (result_index >= 0) neighbors->points[i] = map_data_->points[result_index];
    } 
    else if (refresh_id == "b") {
      map_octree_b_->approxNearestSearch(points.points[i], result_index, unused);
      if (result_index >= 0) neighbors->points[i] = map_data_b_->points[result_index]; 
    }
  }

  return neighbors->points.size() > 0;
}



bool PointCloudMultiThreadedMapper::InsertPoints(const PointCloud::ConstPtr& points, PointCloud* incremental_points) {
  if (!initialized_) {
    ROS_ERROR("%s: Not initialized.", name_.c_str());
    return false;
  }
  if (incremental_points == NULL) {
    ROS_ERROR("%s: Incremental point cloud argument is null.", name_.c_str());
    return false;
  }
  incremental_points->clear();
  // incremental_points->resize(points->points.size()); TODO: put back 

  std::string refresh_id;
  {
    std::lock_guard<std::mutex> lock(refresh_id_mutex_);
    refresh_id = refresh_id_;
  }

  if (refresh_id == "a") {   
    std::lock_guard<std::mutex> lock(map_data_mutex_);
    bool isInBox;
    double min_x, min_y, min_z, max_x, max_y, max_z;
    map_octree_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    for (size_t i = 0; i < points->points.size(); ++i) {
      const Point p = points->points[i];
      isInBox = (p.x >= min_x && p.x <= max_x) &&
                (p.y >= min_y && p.y <= max_y) && 
                (p.z >= min_z && p.z <= max_z);
      if (!isInBox || !map_octree_->isVoxelOccupiedAtPoint(p)) {
        map_octree_->addPointToCloud(p, map_data_);     
        incremental_points->push_back(p);
      }
    } 
  }
  else if (refresh_id == "b") {
    std::lock_guard<std::mutex> lock(map_data_b_mutex_);
    bool isInBox;
    double min_x, min_y, min_z, max_x, max_y, max_z;
    map_octree_b_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    for (size_t i = 0; i < points->points.size(); ++i) {
      const Point p = points->points[i];
      isInBox = (p.x >= min_x && p.x <= max_x) &&
                (p.y >= min_y && p.y <= max_y) && 
                (p.z >= min_z && p.z <= max_z);
      if (!isInBox || !map_octree_b_->isVoxelOccupiedAtPoint(p)) {
        map_octree_b_->addPointToCloud(p, map_data_b_);  
        incremental_points->push_back(p);
      }
    }
  }
       
  if (b_keep_history_) {
    std::lock_guard<std::mutex> lock(history_mutex_);
    for (size_t i = 0; i < incremental_points->points.size(); ++i) {
      const Point p = incremental_points->points[i]; 
      history_->push_back(p); 
    }
  }

  // Publish the incremental map update and return 
  incremental_points->header = points->header;
  incremental_points->header.frame_id = fixed_frame_id_;
  b_inserted_points_ = true;
  map_updated_ = true;
  return true;
}



void PointCloudMultiThreadedMapper::Refresh(const geometry_utils::Transform3& current_pose) {
  Eigen::Vector3f current_translation(3), current_rotation(3);
  current_translation << current_pose.translation.data[0],
                         current_pose.translation.data[1], 
                         current_pose.translation.data[2];
  current_rotation    << current_pose.rotation.Roll(),
                         current_pose.rotation.Pitch(), 
                         current_pose.rotation.Yaw();
  std::lock_guard<std::mutex> lock(box_filter_mutex_); 
  box_filter_.setTranslation(current_translation);
  box_filter_.setRotation(current_rotation);
  b_refresh_ = true; 
}



void PointCloudMultiThreadedMapper::RefreshTimerCallback(const ros::TimerEvent& ev) {
  if (!b_inserted_points_) {
    ROS_WARN("RefreshTimerCallback - returning as !b_inserted_points_");
    return; 
  }
  if (!b_refresh_) {
    return; 
  }

  auto refresh_id = refresh_id_;
  ROS_INFO_STREAM("RefreshTimerCallback with refresh_id: " << refresh_id);
  auto refresh_start_time = std::chrono::system_clock::now();

  if (refresh_id == "a") {
    b_keep_history_ = true;
    {
      std::lock_guard<std::mutex> lock(box_filter_mutex_);
      box_filter_.setInputCloud(map_data_);      
      box_filter_.filter(*map_data_b_);
    }  
    map_octree_b_.reset(new Octree(octree_resolution_));
    map_octree_b_->setInputCloud(map_data_b_);            
    map_octree_b_->addPointsFromInputCloud();   
    {
      std::lock_guard<std::mutex> lock(history_mutex_);
      for (size_t i = 0; i < history_->points.size(); ++i) {
        const Point p = history_->points[i];
        map_octree_b_->addPointToCloud(p, map_data_b_);
      }
      history_->clear(); 
    }  
    b_keep_history_ = false; 
    std::lock_guard<std::mutex> lock(refresh_id_mutex_);
    refresh_id_ = "b";
  } 
  else if (refresh_id == "b") {
    b_keep_history_ = true;
    {
      std::lock_guard<std::mutex> lock(box_filter_mutex_);
      box_filter_.setInputCloud(map_data_b_);     
      box_filter_.filter(*map_data_); 
    }
    map_octree_.reset(new Octree(octree_resolution_));  
    map_octree_->setInputCloud(map_data_);
    map_octree_->addPointsFromInputCloud();       
    {
      std::lock_guard<std::mutex> lock(history_mutex_);
      for (size_t i = 0; i < history_->points.size(); ++i) {
        const Point p = history_->points[i];
        map_octree_->addPointToCloud(p, map_data_);
      }      
      history_->clear();
    }      
    b_keep_history_ = false;
    std::lock_guard<std::mutex> lock(refresh_id_mutex_);
    refresh_id_ = "a";
  }

  auto refresh_end_time = std::chrono::system_clock::now();
  std::chrono::duration<double> refresh_duration = refresh_end_time - refresh_start_time;
  ROS_INFO_STREAM("Refresh duration: " << refresh_duration.count());
  b_refresh_ = false;   
}



void PointCloudMultiThreadedMapper::PublishMap() {
  if (map_pub_.getNumSubscribers() > 0 || !b_publish_only_with_subscribers_) {
    if (initialized_ && map_updated_) {
      if (publish_thread_.joinable()) {
        publish_thread_.join();
      }
      publish_thread_ = std::thread(&PointCloudMultiThreadedMapper::PublishMapThread, this);
    }
  }
}



void PointCloudMultiThreadedMapper::PublishMapThread() {  
  std::string refresh_id; 
  {
    std::lock_guard<std::mutex> lock(refresh_id_mutex_);
    refresh_id = refresh_id_; 
  }
  if (refresh_id == "a") {
    std::lock_guard<std::mutex> lock(map_data_mutex_);
    map_pub_.publish(map_data_);
    map_updated_ = false;
  } 
  else if (refresh_id == "b") { 
    std::lock_guard<std::mutex> lock(map_data_b_mutex_);
    map_pub_.publish(map_data_b_);
    map_b_updated_ = false;
  }
}



void PointCloudMultiThreadedMapper::Reset() {
  ROS_WARN("Not implemented");
}



void PointCloudMultiThreadedMapper::PublishMapFrozen() {
  ROS_WARN("Not implemented");
}



void PointCloudMultiThreadedMapper::PublishMapFrozenThread() {
  ROS_WARN("Not implemented");
}



void PointCloudMultiThreadedMapper::PublishMapInfo() {
  ROS_INFO("Extend default logic to a/b versions of map/octree");
}