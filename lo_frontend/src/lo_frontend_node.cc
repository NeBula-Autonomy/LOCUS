/*
Authors: 
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
*/

#include <ros/ros.h>
#include <lo_frontend/LoFrontend.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lo_frontend_node");
  ros::NodeHandle n("~");

  LoFrontend lo_frontend_node;
  if (!lo_frontend_node.Initialize(n, false)) {
    ROS_ERROR("%s: Failed to initialize lo_frontend_node.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  // Set asynchronous spinners
  std::vector<ros::AsyncSpinner> async_spinners = node.setAsynchSpinners(n);
  for (auto spinner : async_spinners)
    spinner.start();

  // Spin
  ros::spin();

  return EXIT_SUCCESS;
}
