/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for point_cloud_mapper
 */

#include <gtest/gtest.h>

#include "point_cloud_mapper/PointCloudMapper.h"

class PointCloudMapperTest : public ::testing::Test {
protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudMapperTest");
  return RUN_ALL_TESTS();
}
