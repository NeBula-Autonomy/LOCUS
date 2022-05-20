/**
 *  Compute normals from input pointcloud
 *  Matteo Palieri, matteo.palieri@jpl.nasa.gov
 */

#pragma once


#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/search/kdtree.h>
#include <pcl_ros/filters/filter.h>
#include <ros/ros.h>
#include "point_cloud_filter/NormalComputationConfig.h"
namespace point_cloud_filter {

enum SearchNormalsMethod { RADIUS, KNN };
class NormalComputation : public pcl_ros::Filter {
protected:
  typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
  typedef pcl::PointCloud<pcl::Normal> Normals;
  typedef pcl::PointCloud<pcl::PointXYZINormal> PointCloudNormal;

  boost::shared_ptr<dynamic_reconfigure::Server<
      point_cloud_filter::NormalComputationConfig>>
      srv_;

  bool child_init(ros::NodeHandle& nh, bool& has_service);

  void filter(const PointCloud2::ConstPtr& input,
              const IndicesPtr& indices,
              PointCloud2& output);

  void config_callback(point_cloud_filter::NormalComputationConfig& config,
                       uint32_t level);

private:
  pcl::search::KdTree<pcl::PointXYZI>::Ptr search_method_;
  pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> norm_est_;
  SearchNormalsMethod normal_search_method_;
};

} // namespace point_cloud_filter
