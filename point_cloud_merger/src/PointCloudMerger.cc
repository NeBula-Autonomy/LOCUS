/*
 */

#include <point_cloud_merger/PointCloudMerger.h>
#include <parameter_utils/ParameterUtils.h>


namespace pu = parameter_utils;

PointCloudMerger::PointCloudMerger() {}
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

  return true;
}

bool PointCloudMerger::LoadParameters(const ros::NodeHandle& n) {
  // Load mergering parameters.
//   if (!pu::Get("merging/grid_merger", params_.grid_merger)) return false;
//   if (!pu::Get("merging/grid_res", params_.grid_res)) return false;

    if (!pu::Get("merging/decimate_percentage", decimate_percentage_)) return false;


  return true;
}

bool PointCloudMerger::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  pcld1_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nl, "pcld", 10);
  pcld2_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nl, "pcld2", 10);
  pcld_synchronizer = std::unique_ptr<PcldSynchronizer>(
    new PcldSynchronizer(PcldSyncPolicy(pcld_queue_size_), *pcld1_sub_, *pcld2_sub_));
  pcld_synchronizer->registerCallback(&PointCloudMerger::TwoPointCloudCallback, this);

  return true;
}

bool PointCloudMerger::CreatePublishers(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  merged_pcld_pub_ =
      nl.advertise<PointCloud>("combined_point_cloud", 10, false);

  return true;
}

void PointCloudMerger::TwoPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcld1,
                                     const sensor_msgs::PointCloud2::ConstPtr& pcld2) {

  // Merge point clouds
  PointCloud p1, p2;
  pcl::fromROSMsg(*pcld1, p1);
  pcl::fromROSMsg(*pcld2, p2);

  // Simple add together (could do filtering later)
  PointCloud::ConstPtr sum(new PointCloud(p1 + p2));

  // Filter the combined point cloud
  if (b_use_random_filter){
    const int n_points = static_cast<int>((1.0 - decimate_percentage_) *
                                              sum->size());
    pcl::RandomSample<pcl::PointXYZI> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(sum);
    random_filter.filter(*sum);
  }
  // Or a radius filter?

  PublishMergedPointCloud(sum);
}

void PointCloudMerger::PublishMergedPointCloud(const PointCloud::ConstPtr combined_pc){

  // Publish the incoming point cloud message from the reference frame
  if (merged_pcld_pub_.getNumSubscribers() != 0) {
    merged_pcld_pub_.publish(*combined_pc);
  }
}