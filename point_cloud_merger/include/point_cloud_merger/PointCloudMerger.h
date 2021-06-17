#ifndef POINT_CLOUD_MERGER_H
#define POINT_CLOUD_MERGER_H

#include <frontend_utils/CommonStructs.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <parameter_utils/ParameterUtils.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>

class PointCloudMerger {
public:
  PointCloudMerger();
  ~PointCloudMerger();

  bool Initialize(const ros::NodeHandle& n);

  typedef message_filters::Subscriber<PointCloudF>* MessageFilterSub;

  typedef message_filters::sync_policies::ApproximateTime<PointCloudF,
                                                          PointCloudF>
      TwoPcldSyncPolicy;

  typedef message_filters::sync_policies::
      ApproximateTime<PointCloudF, PointCloudF, PointCloudF>
          ThreePcldSyncPolicy;

  typedef message_filters::Synchronizer<TwoPcldSyncPolicy> TwoPcldSynchronizer;
  typedef message_filters::Synchronizer<ThreePcldSyncPolicy>
      ThreePcldSynchronizer;

private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);
  bool CreatePublishers(const ros::NodeHandle& n);

  // TODO: Reduce ----------------------------------------------------------

  void OnePointCloudCallback(const PointCloudF::ConstPtr& a);

  void TwoPointCloudCallback(const PointCloudF::ConstPtr& a,
                             const PointCloudF::ConstPtr& b);

  void ThreePointCloudCallback(const PointCloudF::ConstPtr& a,
                               const PointCloudF::ConstPtr& b,
                               const PointCloudF::ConstPtr& c);

  // -----------------------------------------------------------------------

  void PublishMergedPointCloud(const PointCloudF::ConstPtr combined_pc);

  std::string name_;

  ros::Publisher merged_pcld_pub_;

  int number_of_velodynes_;

  bool b_use_random_filter_;
  double decimate_percentage_;

  bool b_use_radius_filter_;
  double radius_;
  unsigned int radius_knn_;

  int pcld_queue_size_{
      10}; // Approximate time policy queue size to synchronize point clouds

  MessageFilterSub pcld0_sub_;
  MessageFilterSub pcld1_sub_;
  MessageFilterSub pcld2_sub_;

  ros::NodeHandle nl_;
  ros::Subscriber standard_pcld_sub_;

  // TODO: Reduce ----------------------------------------------------------

  std::unique_ptr<TwoPcldSynchronizer> pcld_synchronizer_2_;
  std::unique_ptr<ThreePcldSynchronizer> pcld_synchronizer_3_;

  // -----------------------------------------------------------------------

  /*
  Failure detection --------------------------------------------------------
  Convention:
              - 0:TOP
              - 1:FRONT
              - 2:REAR
  */
  ros::Subscriber failure_detection_sub_;
  ros::Subscriber resurrection_detection_sub_;
  void FailureDetectionCallback(const std_msgs::Int8& sensor_id);
  void ResurrectionDetectionCallback(const std_msgs::Int8& sensor_id);
  int number_of_active_devices_;
  std::map<int, MessageFilterSub> id_to_sub_map_;
  std::vector<int> alive_keys_;
  message_filters::Connection two_sync_connection_;
  message_filters::Connection three_sync_connection_;
  // ------------------------------------------------------------------------
};

#endif
