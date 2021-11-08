/**
 *  Filter robot body from input pointcloud 
 *  Matteo Palieri, matteo.palieri@jpl.nasa.gov
 */

#pragma once

#include <ros/ros.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/filters/filter.h>
#include "point_cloud_filter/BodyFilterConfig.h"



namespace point_cloud_filter {

class BodyFilter : public pcl_ros::Filter {
  
  protected:

    boost::shared_ptr<
        dynamic_reconfigure::Server<point_cloud_filter::BodyFilterConfig>>
        srv_;

    bool child_init(ros::NodeHandle &nh, bool &has_service);

    void filter(const PointCloud2::ConstPtr &input, 
                const IndicesPtr &indices,
                PointCloud2 &output);

    void config_callback(point_cloud_filter::BodyFilterConfig& config,
                         uint32_t level);

  private:
    bool enabled_;
    pcl::CropBox<pcl::PointXYZI> box_filter_;
        
};

}  // namespace point_cloud_filter