/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for point_cloud_merger
 *
 */

#include <gtest/gtest.h>

#include "point_cloud_merger/PointCloudMerger.h"

class PointCloudmergerTest : public ::testing::Test {
protected:
  virtual void SetUp() {
  }

  virtual void TearDown() {
  }
};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudMergerTest");
  return RUN_ALL_TESTS();
}
