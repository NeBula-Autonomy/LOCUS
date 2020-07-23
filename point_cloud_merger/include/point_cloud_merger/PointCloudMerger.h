#ifndef POINT_CLOUD_MERGER_H
#define POINT_CLOUD_MERGER_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <parameter_utils/ParameterUtils.h>

class PointCloudMerger {

  public:

    PointCloudMerger();
    ~PointCloudMerger();

    bool Initialize(const ros::NodeHandle& n);

    typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

  private:
  
    bool LoadParameters(const ros::NodeHandle& n);
    bool RegisterCallbacks(const ros::NodeHandle& n);
    bool CreatePublishers(const ros::NodeHandle& n);

    void TwoPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcld1,
                              const sensor_msgs::PointCloud2::ConstPtr& pcld2);  

    void ThreePointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pcld1,
                                const sensor_msgs::PointCloud2::ConstPtr& pcld2, 
                                const sensor_msgs::PointCloud2::ConstPtr& pcld3);

    void PublishMergedPointCloud(const PointCloud::ConstPtr combined_pc);

    std::string name_;

    ros::Publisher merged_pcld_pub_;

    int number_of_velodynes_;
    
    bool b_use_random_filter_;
    double decimate_percentage_;

    bool b_use_radius_filter_;
    double radius_;
    unsigned int radius_knn_;

    // Approximate time policy queue size to synchronize point clouds
    int pcld_queue_size_{10};
    
    message_filters::Subscriber<sensor_msgs::PointCloud2>* pcld1_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* pcld2_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2>* pcld3_sub_;  
    
    // 2 VLPs
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, 
      sensor_msgs::PointCloud2> PcldSyncPolicy;
    typedef message_filters::Synchronizer<PcldSyncPolicy> PcldSynchronizer;
    std::unique_ptr<PcldSynchronizer> pcld_synchronizer;

    // 3 VLPs
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, 
      sensor_msgs::PointCloud2, 
      sensor_msgs::PointCloud2> PcldSyncPolicy3;
    typedef message_filters::Synchronizer<PcldSyncPolicy3> PcldSynchronizer3;
    std::unique_ptr<PcldSynchronizer3> pcld_synchronizer3;

    // Failure detection subscriber 
    ros::Subscriber failure_detection_sub_;
    void FailureDetectionCallback(const std_msgs::Int8& sensor_id); 

};

#endif
