/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for point_cloud_odometry
 */

#include <gtest/gtest.h>

#include "point_cloud_odometry/PointCloudOdometry.h"

class PointCloudOdometryTest : public ::testing::Test {
protected:
  virtual void SetUp() {
  }

  virtual void TearDown() {
  }
};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudOdometryTest");
  return RUN_ALL_TESTS();
}
