/**
 *  @brief Testing the LaserLoopClosure class
 *
 */

#include <gtest/gtest.h>

#include <math.h>
#include <ros/ros.h>

#include <pose_graph_msgs/KeyedScan.h>
#include <pose_graph_msgs/PoseGraph.h>
#include <pose_graph_msgs/PoseGraphEdge.h>
#include <pose_graph_msgs/PoseGraphNode.h>

#include <gtsam/inference/Key.h>
#include <gtsam/inference/Symbol.h>

#include <lo_frontend/LoFrontend.h>

class TestLoFrontend : public ::testing::Test {
  public:
    TestLoFrontend() :
      lc(ros::NodeHandle()) {
      // Set params
    }
    ~TestLoFrontend() {}

    
    TestLoFrontend lo;

  protected:


  private:
};

TEST_F(TestLoFrontend, TestLoFrontend_init) {

  // initialize
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lo_frontend");
  return RUN_ALL_TESTS();
}