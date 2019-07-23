/**
 *  @Author Abhishek Thakur
 *  @brief Test cases for measurement_synchronizer class
 *
 */

#include <gtest/gtest.h>
#include "measurement_synchronizer/MeasurementSynchronizer.h"

class MeasurementSynchronizerTest : public ::testing::Test {
protected:
  virtual void SetUp() {
  }

  virtual void TearDown() {
  }

};

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "MeasurementSynchronizerTest");
  return RUN_ALL_TESTS();
}
