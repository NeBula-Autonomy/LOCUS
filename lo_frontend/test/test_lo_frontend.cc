/**
 *  @brief Testing the LaserLoopClosure class
 *
 */

#include <gtest/gtest.h>

#include <math.h>
#include <ros/ros.h>

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

#include <lo_frontend/LoFrontend.h>

class TestLoFrontend : public ::testing::Test {
  public:
    TestLoFrontend() {
      // Set params
    }
    ~TestLoFrontend() {}

    
    LoFrontend lo;

  protected:


  private:
};

TEST_F(TestLoFrontend, TestLoFrontend_init) {
  ros::NodeHandle n("~");
  // initialize
  lo.Initialize(n, false);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lo_frontend");
  return RUN_ALL_TESTS();
}