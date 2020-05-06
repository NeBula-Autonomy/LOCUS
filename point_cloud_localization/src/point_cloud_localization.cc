/*
Authors: 
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
*/

#include <ros/ros.h>
#include <pcl/search/impl/search.hpp>
#include <point_cloud_localization/PointCloudLocalization.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_localization");
  ros::NodeHandle n("~");

  PointCloudLocalization pcl;
  if (!pcl.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud localization.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
