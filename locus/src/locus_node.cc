/*
Authors:
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
*/

#include <locus/Locus.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "locus_node");
  ros::NodeHandle n("~");

  Locus locus_node;
  if (!locus_node.Initialize(n, false)) {
    ROS_ERROR("%s: Failed to initialize locus_node.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  // Set asynchronous spinners
  std::vector<ros::AsyncSpinner> async_spinners =
      locus_node.setAsynchSpinners(n);
  for (auto spinner : async_spinners)
    spinner.start();

  // Spin
  ros::spin();

  return EXIT_SUCCESS;
}
