/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for point_cloud_localization class
 */

#include <gtest/gtest.h>

#include "point_cloud_localization/PointCloudLocalization.h"

class PointCloudLocalizationTest : public ::testing::Test {
protected:
  virtual void SetUp() {
  }

  virtual void TearDown() {
  }
};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudLocalizationTest");
  return RUN_ALL_TESTS();
}
