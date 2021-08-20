/*
Authors:
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
*/

#include <gtest/gtest.h>
#include <lo_frontend/LoFrontend.h>

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
    system("rosparam set b_interpolate false");
    ros::param::set("mapper/num_threads", 2);
  }

  ~LoFrontendTest() {}

  LoFrontend lf;

protected:
  // bool SetDataIntegrationMode() {
  //   return lf.SetDataIntegrationMode();
  // }

  void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    lf.ImuCallback(imu_msg);
  }

  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg) {
    lf.OdometryCallback(odometry_msg);
  }

  // void PoseStampedCallback(
  //     const geometry_msgs::PoseStamped::ConstPtr& pose_stamped_msg) {
  //   lf.PoseStampedCallback(pose_stamped_msg);
  // }

  tf::Transform GetOdometryDelta(const tf::Transform& odometry_pose) const {
    return lf.GetOdometryDelta(odometry_pose);
  }

  Eigen::Matrix3d GetImuDelta() {
    return lf.GetImuDelta();
  }

  Eigen::Matrix3d GetImuYawDelta() {
    return lf.GetImuYawDelta();
  }

  // void SwitchToImuIntegration() {
  //   lf.SwitchToImuIntegration();
  // }

private:
};

/* TEST Initialize */
TEST_F(LoFrontendTest, TestInitialize) {
  ros::NodeHandle nh;
  bool result = lf.Initialize(nh, false);
  ASSERT_TRUE(result);
}

// /* TEST SetDataIntegratioMode*/
// TEST_F(LoFrontendTest, TestSetDataIntegrationMode) {
//   ros::NodeHandle nh;
//   lf.Initialize(nh, false);
//   bool result = SetDataIntegrationMode();
//   ASSERT_TRUE(result);
// }

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_lo_frontend");
  return RUN_ALL_TESTS();
}
