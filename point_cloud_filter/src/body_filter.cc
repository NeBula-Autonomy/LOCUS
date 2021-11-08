/**
 *  Filter robot body from input pointcloud 
 *  Matteo Palieri, matteo.palieri@jpl.nasa.gov
 */

#include <pluginlib/class_list_macros.h>
#include "point_cloud_filter/body_filter.h"



namespace point_cloud_filter {

bool BodyFilter::child_init(ros::NodeHandle &nh, bool &has_service) {
  enabled_ = false;
  has_service = true;
  srv_ = boost::make_shared<dynamic_reconfigure::Server<
      point_cloud_filter::BodyFilterConfig>>(nh);
  dynamic_reconfigure::Server<
      point_cloud_filter::BodyFilterConfig>::CallbackType f =
      boost::bind(&BodyFilter::config_callback, this, _1, _2);
  srv_->setCallback(f);

  return true;

}

void BodyFilter::filter(const PointCloud2::ConstPtr &input,
                              const IndicesPtr &indices,
                              PointCloud2 &output) {
  if (!enabled_) {
    output = *input;
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>());
  pcl::fromROSMsg(*input, *input_cloud);
  box_filter_.setInputCloud(input_cloud); 
  box_filter_.filter(*input_cloud);
  pcl::toROSMsg(*input_cloud, output);

}

void BodyFilter::config_callback(
    point_cloud_filter::BodyFilterConfig& config, uint32_t level) {
  enabled_ = config.enabled;
  box_filter_.setMin(
      Eigen::Vector4f(config.min_x, config.min_y, config.min_z, 1.0));
  box_filter_.setMax(
      Eigen::Vector4f(config.max_x, config.max_y, config.max_z, 1.0));
  box_filter_.setRotation(Eigen::Vector3f(0.0, 0.0, config.rotation));
  box_filter_.setNegative(true);
}

}  // namespace point_cloud_filter

PLUGINLIB_EXPORT_CLASS(point_cloud_filter::BodyFilter, nodelet::Nodelet)