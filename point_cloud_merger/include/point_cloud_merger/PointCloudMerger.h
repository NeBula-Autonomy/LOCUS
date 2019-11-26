/*
 */

#ifndef POINT_CLOUD_MERGER_H
#define POINT_CLOUD_MERGER_H

//ROS
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class PointCloudMerger {
 public:
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr pclConstPtr;
  typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pclPtr;

  PointCloudMerger();
  ~PointCloudMerger();

  // Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
  bool Initialize(const ros::NodeHandle& n);



 private:

  // Node initialization.
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  // Callback
  void TwoPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcld1,
                                     const sensor_msgs::PointCloud2::ConstPtr& pcld2) ;

  // Publish
  void PublishMergedPointCloud(const PointCloud::ConstPtr combined_pc);

  // The node's name.
  std::string name_;

  // Subscribers.
  // Queue size of the approximate time policy that synchronizes the two point clouds.
  int pcld_queue_size_{10};
  // Filters
  message_filters::Subscriber<sensor_msgs::PointCloud2>* pcld1_sub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* pcld2_sub_;
  // Synchronization policy for the two point clouds.
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> PcldSyncPolicy;
  // Synchronizer for the two point clouds.
  typedef message_filters::Synchronizer<PcldSyncPolicy> PcldSynchronizer;
  std::unique_ptr<PcldSynchronizer> pcld_synchronizer;

  // Publishers
  ros::Publisher merged_pcld_pub_;

  double decimate_percentage_;
  bool b_use_random_filter_;

  // Apply a radius outlier filter.
  bool b_use_radius_filter_;
  // Size of the radius filter.
  double radius_;
  // If this number of neighbors are not found within a radius around each
  // point, remove that point.
  unsigned int radius_knn_;


  // struct Parameters {
  //   // Apply feature extraction
  //   bool extract_features;
  //   // Apply a voxel grid merger.
  //   bool grid_merger;
  //   // Resolution of voxel grid merger.
  //   double grid_res;
  //   // Apply a random downsampling merger.
  //   bool random_merger;
  //   // Percentage of points to discard. Must be between 0.0 and 1.0;
  //   double decimate_percentage;
  //   // Apply a statistical outlier merger.
  //   bool outlier_merger;
  //   // Standard deviation threshold in distance to neighbors for outlier
  //   // removal.
  //   double outlier_std;
  //   // Number of nearest neighbors to use for outlier merger.
  //   unsigned int outlier_knn;
  //   // Apply a radius outlier merger.
  //   bool radius_merger;
  //   // Size of the radius merger.
  //   double radius;
  //   // If this number of neighbors are not found within a radius around each
  //   // point, remove that point.
  //   unsigned int radius_knn;
  // } params_;

};

#endif
