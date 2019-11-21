/*
 */

#include <ros/ros.h>
#include <point_cloud_merger/PointCloudMerger.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_merger");
  ros::NodeHandle n("~");

  PointCloudMerger pcm;
  if (!pcm.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize point cloud merger.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();

  return EXIT_SUCCESS;
}
