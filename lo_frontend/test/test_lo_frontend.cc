#include <lo_frontend/LoFrontend.h>
#include <gtest/gtest.h>

class LoFrontendTest : public ::testing::Test {
  
  public:

    LoFrontendTest() {

      system("rosparam load $(rospack find "
             "lo_frontend)/config/lo_frames.yaml");  
      system("rosparam load $(rospack find "
             "lo_frontend)/config/lo_settings.yaml");  
      system("rosparam load $(rospack find "
             "lidar_slip_detection)/config/parameters.yaml");   
      system("rosparam load $(rospack find "
             "point_cloud_merger)/config/parameters.yaml"); 
      system("rosparam load $(rospack find "
             "point_cloud_filter)/config/parameters.yaml"); 
      system("rosparam load $(rospack find "
             "point_cloud_odometry)/config/parameters.yaml"); 
      system("rosparam load $(rospack find "
             "point_cloud_localization)/config/parameters.yaml"); 
      system("rosparam load $(rospack find "
             "point_cloud_mapper)/config/parameters.yaml"); 
      system("rosparam set icp/num_threads 1");
      system("rosparam set localization/num_threads 1");
      
    }

    ~LoFrontendTest() {}
    
    LoFrontend lf;

  protected:

  private:

};

/* TEST Initialize */
TEST_F(LoFrontendTest, TestInitialize) {
  ros::NodeHandle nh;
  bool result = lf.Initialize(nh, false);
  ASSERT_TRUE(result);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lo_frontend");
  return RUN_ALL_TESTS();
}