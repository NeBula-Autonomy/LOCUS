/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for point_cloud_filter
 *
 */

#include <gtest/gtest.h>

#include "point_cloud_filter/PointCloudFilter.h"

class PointCloudFilterTest : public ::testing::Test {
protected:
  virtual void SetUp() {
  }

  virtual void TearDown() {
  }
};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudFilterTest");
  return RUN_ALL_TESTS();
}
