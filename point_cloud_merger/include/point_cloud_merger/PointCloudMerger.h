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
    
    typedef pcl::PointXYZI Point; 
    typedef pcl::PointCloud<Point> PointCloud;
    typedef message_filters::Subscriber<sensor_msgs::PointCloud2>* MessageFilterSub; 
    
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, 
      sensor_msgs::PointCloud2> TwoPcldSyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2, 
      sensor_msgs::PointCloud2, 
      sensor_msgs::PointCloud2> ThreePcldSyncPolicy;

    typedef message_filters::Synchronizer<TwoPcldSyncPolicy> TwoPcldSynchronizer;
    typedef message_filters::Synchronizer<ThreePcldSyncPolicy> ThreePcldSynchronizer;

  private:
  
    bool LoadParameters(const ros::NodeHandle& n);
    bool RegisterCallbacks(const ros::NodeHandle& n);
    bool CreatePublishers(const ros::NodeHandle& n);

    // TODO: Reduce ----------------------------------------------------------

    void OnePointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& a);
    
    void TwoPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& a,
                               const sensor_msgs::PointCloud2::ConstPtr& b);  

    void ThreePointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& a,
                                 const sensor_msgs::PointCloud2::ConstPtr& b, 
                                 const sensor_msgs::PointCloud2::ConstPtr& c);

    // -----------------------------------------------------------------------

    void PublishMergedPointCloud(const PointCloud::ConstPtr combined_pc);

    std::string name_;

    ros::Publisher merged_pcld_pub_;

    int number_of_velodynes_;
    
    bool b_use_random_filter_;
    double decimate_percentage_;

    bool b_use_radius_filter_;
    double radius_;
    unsigned int radius_knn_;

    int pcld_queue_size_{10}; // Approximate time policy queue size to synchronize point clouds
  
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
    // ------------------------------------------------------------------------

};

#endif
