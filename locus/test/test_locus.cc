/*
Authors:
  - Matteo Palieri    (matteo.palieri@jpl.nasa.gov)
  - Benjamin Morrell  (benjamin.morrell@jpl.nasa.gov)
*/

#include <gtest/gtest.h>
#include <locus/Locus.h>

class LocusTest : public ::testing::Test {
public:
  LocusTest() {
    system("rosparam load $(rospack find "
           "locus)/config/lo_frames.yaml");
    system("rosparam load $(rospack find "
           "locus)/config/lo_settings.yaml");
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
    system("rosparam set b_integrate_interpolated_odom false");
    system("rosparam set b_pub_odom_on_timer false");
    ros::param::set("mapper/num_threads", 2);
  }

  ~LocusTest() {}

  Locus lf;

protected:
  void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    lf.ImuCallback(imu_msg);
  }

  void OdometryCallback(const nav_msgs::Odometry::ConstPtr& odometry_msg) {
    lf.OdometryCallback(odometry_msg);
  }

  tf::Transform GetOdometryDelta(const tf::Transform& odometry_pose) const {
    return lf.GetOdometryDelta(odometry_pose);
  }

  Eigen::Matrix3d GetImuDelta() {
    return lf.GetImuDelta();
  }

  Eigen::Matrix3d GetImuYawDelta() {
    return lf.GetImuYawDelta();
  }

private:
};

/* TEST Initialize */
TEST_F(LocusTest, TestInitialize) {
  ros::NodeHandle nh;
  bool result = lf.Initialize(nh, false);
  ASSERT_TRUE(result);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_locus");
  return RUN_ALL_TESTS();
}