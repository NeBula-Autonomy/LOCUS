/*
Authors: 
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
*/

#include <ros/ros.h>
#include <spot_frontend/SpotFrontend.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "spot_frontend_node");
  ros::NodeHandle n("~");
  SpotFrontend spot_frontend_node;
  if (!spot_frontend_node.Initialize(n, false)) {
    ROS_ERROR("%s: Failed to initialize spot_frontend_node.", 
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }
  ros::spin();
  return EXIT_SUCCESS;
}
