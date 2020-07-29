#include <point_cloud_merger/PointCloudMerger.h>

namespace pu = parameter_utils;

PointCloudMerger::PointCloudMerger(): 
  b_use_random_filter_(false),
  b_use_radius_filter_(false) {}

PointCloudMerger::~PointCloudMerger() {}

bool PointCloudMerger::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudMerger");
  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }
  if (!CreatePublishers(n)) {
    ROS_ERROR("%s: Failed to create publishers.", name_.c_str());
    return false;
  }
  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }
  number_of_active_devices_ = number_of_velodynes_;
  return true;
}

bool PointCloudMerger::LoadParameters(const ros::NodeHandle& n) {
  if (!pu::Get("merging/number_of_velodynes", number_of_velodynes_)) 
    return false;
  if (!pu::Get("merging/decimate_percentage", decimate_percentage_)) 
    return false;
  if (!pu::Get("merging/b_use_random_filter", b_use_random_filter_)) 
    return false;
  if (!pu::Get("merging/b_use_radius_filter", b_use_radius_filter_)) 
    return false;
  if (!pu::Get("merging/radius", radius_)) 
    return false;
  if (!pu::Get("merging/radius_knn", radius_knn_)) 
    return false;
  return true;
}

bool PointCloudMerger::RegisterCallbacks(const ros::NodeHandle& n) {

  nl_ = ros::NodeHandle(n);

  failure_detection_sub_ = nl_.subscribe("failure_detection", 10, &PointCloudMerger::FailureDetectionCallback, this); 

  pcld0_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nl_, "pcld0", 10);
  pcld1_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nl_, "pcld1", 10);  

  if (number_of_velodynes_==2) {
    ROS_INFO("PointCloudMerger - 2 VLPs merging requested");
    pcld_synchronizer_2_ = std::unique_ptr<PcldSynchronizer2>(
      new PcldSynchronizer2(PcldSyncPolicy2(pcld_queue_size_), *pcld0_sub_, *pcld1_sub_));
    pcld_synchronizer_2_->registerCallback(&PointCloudMerger::TwoPointCloudCallback, this);
  }
  else if (number_of_velodynes_==3) {
    ROS_INFO("PointCloudMerger - 3 VLPs merging requested");
    pcld2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nl_, "pcld2", 10);
    pcld_synchronizer_3_ = std::unique_ptr<PcldSynchronizer3>(
      new PcldSynchronizer3(PcldSyncPolicy3(pcld_queue_size_), *pcld0_sub_, *pcld1_sub_, *pcld2_sub_));
    pcld_synchronizer_3_->registerCallback(&PointCloudMerger::ThreePointCloudCallback, this);
    
    id_to_sub_map_.insert({ 0, pcld0_sub_});
    id_to_sub_map_.insert({ 1, pcld1_sub_});
    id_to_sub_map_.insert({ 2, pcld2_sub_});
  
  }
  else {
    ROS_WARN("PointCloudMerger - number_of_velodynes_ !=2 and !=3");
    return false;
  }
 
  return true;

}

bool PointCloudMerger::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  merged_pcld_pub_ = nl.advertise<PointCloud>("combined_point_cloud", 10, false);
  return true;
}





// TODO: Reduce---------------------------------------------------------------------------------

void PointCloudMerger::OnePointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& a) {

  PointCloud p1;
  pcl::fromROSMsg(*a, p1);
  PointCloud::Ptr cloud(new PointCloud(p1));

  if (b_use_random_filter_){
    const int n_points = static_cast<int>((1.0 - decimate_percentage_) * cloud->size());
    pcl::RandomSample<pcl::PointXYZI> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(cloud);
    random_filter.filter(*cloud);
  }
  if (b_use_radius_filter_){
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad;
    rad.setInputCloud(cloud);
    rad.setRadiusSearch(radius_);
    rad.setMinNeighborsInRadius(radius_knn_);
    rad.filter(*cloud);
  }

  PublishMergedPointCloud(cloud);

}

void PointCloudMerger::TwoPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& a,
                                             const sensor_msgs::PointCloud2::ConstPtr& b) {

  PointCloud p1, p2;
  pcl::fromROSMsg(*a, p1);
  pcl::fromROSMsg(*b, p2);

  PointCloud::Ptr sum(new PointCloud(p1 + p2));

  if (b_use_random_filter_){
    const int n_points = static_cast<int>((1.0 - decimate_percentage_) * sum->size());
    pcl::RandomSample<pcl::PointXYZI> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(sum);
    random_filter.filter(*sum);
  }
  if (b_use_radius_filter_){
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad;
    rad.setInputCloud(sum);
    rad.setRadiusSearch(radius_);
    rad.setMinNeighborsInRadius(radius_knn_);
    rad.filter(*sum);
  }

  PublishMergedPointCloud(sum);

}

void PointCloudMerger::ThreePointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& a,
                                               const sensor_msgs::PointCloud2::ConstPtr& b, 
                                               const sensor_msgs::PointCloud2::ConstPtr& c) {
  
  PointCloud p1, p2, p3;
  pcl::fromROSMsg(*a, p1);
  pcl::fromROSMsg(*b, p2);
  pcl::fromROSMsg(*c, p3);

  PointCloud::Ptr sum(new PointCloud(p1 + (p2 + p3)));
  
  if (b_use_random_filter_){
    const int n_points = static_cast<int>((1.0 - decimate_percentage_) * sum->size());
    pcl::RandomSample<pcl::PointXYZI> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(sum);
    random_filter.filter(*sum);
  }
  if (b_use_radius_filter_){
    pcl::RadiusOutlierRemoval<pcl::PointXYZI> rad;
    rad.setInputCloud(sum);
    rad.setRadiusSearch(radius_);
    rad.setMinNeighborsInRadius(radius_knn_);
    rad.filter(*sum);
  }

  PublishMergedPointCloud(sum);

}

// ---------------------------------------------------------------------------------------------





void PointCloudMerger::PublishMergedPointCloud(const PointCloud::ConstPtr combined_pc) {
  if (merged_pcld_pub_.getNumSubscribers() != 0) merged_pcld_pub_.publish(*combined_pc);
}





void PointCloudMerger::FailureDetectionCallback(const std_msgs::Int8& sensor_id) {
  
  /*
  CONVENTION:
     - 0:TOP
     - 1:FRONT
     - 2:REAR
  
  NOTE: 
    - First Basic Implementation
    - Preliminary assumption is: dead once, dead forever  

  TODO: 
    - Implement ResurrectionDetection as well in sensors_health_monitor.py 
    - Minimize number of i) synchronizers and ii) PointCloudCallback
  */
  
  ROS_INFO("PointCloudMerger - Received failure detection of sensor %d", sensor_id.data);
  number_of_active_devices_--; 

  // TODO: Generalize [now assuming to kill: i) REAR, then ii) FRONT]

  if (number_of_active_devices_ == 2) {
    pcld_synchronizer_2_ = std::unique_ptr<PcldSynchronizer2>(
        new PcldSynchronizer2(PcldSyncPolicy2(pcld_queue_size_), *pcld0_sub_, *pcld1_sub_));
    pcld_synchronizer_2_->registerCallback(&PointCloudMerger::TwoPointCloudCallback, this);
  }
  else if (number_of_active_devices_ == 1) {
    standard_pcld_sub_ = nl_.subscribe("pcld0", 1, &PointCloudMerger::OnePointCloudCallback, this);
  } 
  else if (number_of_active_devices_ == 0) {
    ROS_ERROR("PointCloudMerger - No active lidar sensors");
  }
     
} 