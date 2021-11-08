/**
 *  Compute normals from input pointcloud
 *  Matteo Palieri, matteo.palieri@jpl.nasa.gov
 *  Andrzej Reinke, andrzej.m.reinke@jpl.nasa.gov
 */

#include "point_cloud_filter/normal_computation.h"
#include <pluginlib/class_list_macros.h>

namespace point_cloud_filter {

bool NormalComputation::child_init(ros::NodeHandle& nh, bool& has_service) {
  norm_est_.setSearchMethod(search_method_);

  has_service = true;
  srv_ = boost::make_shared<dynamic_reconfigure::Server<
      point_cloud_filter::NormalComputationConfig>>(nh);
  dynamic_reconfigure::Server<
      point_cloud_filter::NormalComputationConfig>::CallbackType f =
      boost::bind(&NormalComputation::config_callback, this, _1, _2);
  srv_->setCallback(f);

  return true;
}

void NormalComputation::filter(const PointCloud2::ConstPtr& input,
                               const IndicesPtr& indices,
                               PointCloud2& output) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*input, *input_cloud);

  Normals::Ptr normals(new Normals());

  norm_est_.setInputCloud(input_cloud);
  norm_est_.compute(*normals);

  PointCloudNormal::Ptr output_cloud_with_normals(new PointCloudNormal());
  for (size_t i = 0; i < input_cloud->points.size(); i++) {
    pcl::PointXYZINormal point;
    point.x = input_cloud->points[i].x;
    point.y = input_cloud->points[i].y;
    point.z = input_cloud->points[i].z;
    point.intensity = input_cloud->points[i].intensity;
    point.normal_x = normals->at(i).normal_x;
    point.normal_y = normals->at(i).normal_y;
    point.normal_z = normals->at(i).normal_z;
    output_cloud_with_normals->points.push_back(point);
  }
  output_cloud_with_normals->header = input_cloud->header;

  if (normal_search_method_ == SearchNormalsMethod::RADIUS) {
    std::vector<int> not_used;
    pcl::removeNaNNormalsFromPointCloud(
        *output_cloud_with_normals, *output_cloud_with_normals, not_used);
  }

  pcl::toROSMsg(*output_cloud_with_normals, output);
}

void NormalComputation::config_callback(
    point_cloud_filter::NormalComputationConfig& config, uint32_t level) {
  ROS_INFO_STREAM("Normal computation in the nodelet.");
  ROS_INFO_STREAM("No threads: " << config.num_threads);
  if (config.normal_search_method == "search_knn") {
    ROS_INFO_STREAM("search knn: " << config.normal_search_knn);
    norm_est_.setRadiusSearch(0);
    norm_est_.setKSearch(config.normal_search_knn);
    normal_search_method_ = SearchNormalsMethod::KNN;
  } else if (config.normal_search_method == "search_radius") {
    ROS_INFO_STREAM("SEARCH RADIUS" << config.normal_search_radius);
    norm_est_.setKSearch(0);
    norm_est_.setRadiusSearch(config.normal_search_radius);
    normal_search_method_ = SearchNormalsMethod::RADIUS;
  } else {
    ROS_ERROR_STREAM("Wrong normal search method in "
                     "point_cloud_filter/NormalComputation");
    EXIT_FAILURE;
  }
  norm_est_.setNumberOfThreads(config.num_threads);
}

} // namespace point_cloud_filter

PLUGINLIB_EXPORT_CLASS(point_cloud_filter::NormalComputation,
                       nodelet::Nodelet)
