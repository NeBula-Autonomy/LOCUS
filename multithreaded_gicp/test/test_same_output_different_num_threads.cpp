#include <multithreaded_gicp/gicp.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// TODO(JN) refactor this into a gtest
int main(int argc, char **argv) {
  // Read in point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr query (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr reference (new pcl::PointCloud<pcl::PointXYZI>);

  std::string package_path = ros::package::getPath("multithreaded_gicp");
  
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (package_path + "/test/query_82_garage.pcd", *query) == -1) {
    std::cerr << "Could not load query_82_garage.pcd" << std::endl;
    return -1;
  }
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (package_path + "/test/reference_82_garage.pcd", *reference) == -1) {
    std::cerr << "Could not load reference_82_garage.pcd" << std::endl;
    return -1;
  }
  
  // Test gicp with num threads = 1 to 8
  for (int i = 1; i < 9; i++) {
    pcl::MultithreadedGeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp; // (i, true);
    icp.setTransformationEpsilon(0.0000000001);
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(20);
    icp.setRANSACIterations(0);
    icp.setMaximumOptimizerIterations(50);
    icp.setNumThreads(i);
    icp.enableTimingOutput(true);
    icp.setInputSource(query);
    icp.setInputTarget(reference);
    pcl::PointCloud<pcl::PointXYZI> aligned_points;
    icp.align(aligned_points);
    const Eigen::Matrix4f T = icp.getFinalTransformation();
    std::cout << "Fitness score: " << icp.getFitnessScore() << std::endl;
  }
}

