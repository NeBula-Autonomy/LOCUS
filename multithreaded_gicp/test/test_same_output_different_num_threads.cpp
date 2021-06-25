#include <frontend_utils/CommonStructs.h>
#include <multithreaded_gicp/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <ros/package.h>

// TODO(JN) refactor this into a gtest
int main(int argc, char** argv) {
  // Read in point clouds
  PointCloudF::Ptr query(new PointCloudF);
  PointCloudF::Ptr reference(new PointCloudF);

  std::string package_path = ros::package::getPath("multithreaded_gicp");

  if (pcl::io::loadPCDFile<PointF>(package_path + "/test/query_82_garage.pcd",
                                   *query) == -1) {
    std::cerr << "Could not load query_82_garage.pcd" << std::endl;
    return -1;
  }
  if (pcl::io::loadPCDFile<PointF>(
          package_path + "/test/reference_82_garage.pcd", *reference) == -1) {
    std::cerr << "Could not load reference_82_garage.pcd" << std::endl;
    return -1;
  }

  double original_gicp_fitness_score;
  Eigen::Matrix4f original_gicp_transform;
  {
    std::cout << "Original GICP" << std::endl;
    pcl::GeneralizedIterativeClosestPoint<PointF, PointF> icp;
    icp.setTransformationEpsilon(0.0000000001);
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(20);
    icp.setRANSACIterations(0);
    icp.setMaximumOptimizerIterations(50);
    icp.setInputSource(query);
    icp.setInputTarget(reference);
    PointCloudF aligned_points;
    icp.align(aligned_points);
    original_gicp_transform = icp.getFinalTransformation();
    std::cout << original_gicp_transform << std::endl;
    original_gicp_fitness_score = icp.getFitnessScore();
    std::cout << "Fitness score: " << original_gicp_fitness_score << std::endl;
    std::cout << std::endl;
  }

  bool success = true;

  // Test gicp with num threads = 1 to 8
  for (int i = 1; i < 9; i++) {
    pcl::MultithreadedGeneralizedIterativeClosestPoint<PointF, PointF> icp;
    icp.setTransformationEpsilon(0.0000000001);
    icp.setMaxCorrespondenceDistance(0.2);
    icp.setMaximumIterations(20);
    icp.setRANSACIterations(0);
    icp.setMaximumOptimizerIterations(50);
    icp.setNumThreads(i);
    icp.enableTimingOutput(true);
    icp.setInputSource(query);
    icp.setInputTarget(reference);
    PointCloudF aligned_points;
    icp.align(aligned_points);
    const Eigen::Matrix4f T = icp.getFinalTransformation();
    std::cout << T << std::endl;
    double fitness_score = icp.getFitnessScore();
    std::cout << "Fitness score: " << fitness_score << std::endl;
    if (T == original_gicp_transform) {
      std::cout << "SUCCESS: Transform matches original GICP transform"
                << std::endl;
    } else {
      std::cerr << "FAILURE: Transform does not match original GICP transform"
                << std::endl;
      success = false;
    }

    if (fitness_score == original_gicp_fitness_score) {
      std::cout << "SUCCESS: Fitness score matches original GICP fitness score"
                << std::endl;
    } else {
      std::cerr
          << "FAILURE: Fitness score does not match original GICP fitness score"
          << std::endl;
      success = false;
    }
    std::cout << std::endl;
  }

  if (success) {
    std::cout << "All checks passed!" << std::endl;
  } else {
    std::cerr << "At least one check failed" << std::endl;
  }
}
