/**
 *  @author Abhishek Thakur
 *  @author Andrzej Reinke
 *  @brief Test cases for point_cloud_odometry
 * The following functions don't have unitests:
 *  GetIntegratedEstimate - too simple
 *  GetIncrementalEstimate - too simple
 *  PublishAll - ros specific
 *  PublishPose - ros specific
 * /////////////////////////////////////////////////////////////////////////
 *  TODO: decide whether GenerateCubic, GenerateHollowCubic, CreateBox and their
 * tests should be here or in a test_common repo (that currently doesn't exist)
 */

#include <gtest/gtest.h>

#include "point_cloud_odometry/PointCloudOdometry.h"

// TODO: add tests for ndt
const float epsilion = 1e-2f;
const double epsiliond = 1e-2;

PointCloudF::Ptr GenerateCubic(size_t x_points = 10,
                               size_t y_points = 10,
                               size_t z_points = 10,
                               float step_x = 0.1f,
                               float step_y = 0.1f,
                               float step_z = 0.1f,
                               std::string frame_name = "dummy") {
  auto pc_out = boost::make_shared<PointCloudF>();
  pc_out->reserve(x_points * y_points * z_points);
  pc_out->header.frame_id = frame_name;

  for (size_t ix = 0; ix < x_points; ix++) {
    for (size_t iy = 0; iy < y_points; iy++) {
      for (size_t iz = 0; iz < z_points; iz++) {
        pc_out->push_back(PointF({ix * step_x, iy * step_y, iz * step_z, 0}));
      }
    }
  }
  pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal> ne;
  ne.setInputCloud(pc_out);
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZINormal>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(5);
  ne.compute(*cloud_normals);

  for (size_t i = 0; i < pc_out->size(); ++i) {
    pc_out->points[i].normal_x = cloud_normals->points[i].normal_x;
    pc_out->points[i].normal_y = cloud_normals->points[i].normal_y;
    pc_out->points[i].normal_z = cloud_normals->points[i].normal_z;
  }
  return pc_out;
}

PointCloudF::Ptr GenerateHollowCubic(size_t x_points = 10,
                                     size_t y_points = 10,
                                     size_t z_points = 10,
                                     float step_x = 0.1f,
                                     float step_y = 0.1f,
                                     float step_z = 0.1f,
                                     std::string frame_name = "dummy") {
  auto pc_out = boost::make_shared<PointCloudF>();
  pc_out->reserve(x_points * y_points * z_points);
  pc_out->header.frame_id = frame_name;

  for (size_t ix = 0; ix < x_points; ix++) {
    for (size_t iy = 0; iy < y_points; iy++) {
      for (size_t iz = 0; iz < z_points; iz++) {
        if (ix == 0 || iy == 0 || ix == x_points - 1 || iy == y_points - 1) {
          pc_out->push_back(PointF({ix * step_x, iy * step_y, iz * step_z, 0}));
        }
      }
    }
  }
  pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal> ne;
  ne.setInputCloud(pc_out);
  pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZINormal>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(
      new pcl::PointCloud<pcl::Normal>);
  ne.setKSearch(5);
  ne.compute(*cloud_normals);

  for (size_t i = 0; i < pc_out->size(); ++i) {
    pc_out->points[i].normal_x = cloud_normals->points[i].normal_x;
    pc_out->points[i].normal_y = cloud_normals->points[i].normal_y;
    pc_out->points[i].normal_z = cloud_normals->points[i].normal_z;
  }
  return pc_out;
}

PointCloudF::Ptr
CreateBox(float width = 1.0f, float height = 1.0f, float depth = 1.0f) {
  size_t points = 8;
  auto pc = boost::make_shared<PointCloudF>();
  if (width <= 0) {
    throw std::runtime_error("[CreateBox] width <= 0");
  }
  if (height <= 0) {
    throw std::runtime_error("[CreateBox] height <= 0");
  }
  if (depth <= 0) {
    throw std::runtime_error("[CreateBox] depth <= 0");
  }
  pc->points.reserve(points);

  pc->push_back(PointF({0.0, 0.0, 0.0, 0.0}));
  pc->push_back(PointF({width, 0.0, 0.0, 0.0}));
  pc->push_back(PointF({0.0, 0.0, depth, 0.0}));
  pc->push_back(PointF({width, 0.0, depth, 0.0}));
  pc->push_back(PointF({0.0, height, 0.0, 0.0}));
  pc->push_back(PointF({width, height, 0.0, 0.0}));
  pc->push_back(PointF({0.0, height, depth, 0.0}));
  pc->push_back(PointF({width, height, depth, 0.0}));

  return pc;
}

// TODO: maybe it should be moved somewhere to common pkg
TEST(CreateBoxTest, TestCreateBox) {
  auto box = CreateBox(5.0f, 2.0f, 3.0f);
  EXPECT_EQ(box->points.size(), 8);
}
// TODO: maybe it should be moved somewhere to common pkg
TEST(CreateCubixTest, TestCreateCubic) {
  auto cubic = GenerateCubic();
  EXPECT_EQ(cubic->points.size(), 1000);
}

class PointCloudOdometryTest : public ::testing::Test {
public:
  size_t n_points_;
  PointCloudOdometry pco;
  PointCloudF::Ptr data;

  PointCloudOdometryTest()
    : n_points_(30), data(new PointCloudF(n_points_, 1)) {
    std::system("rosparam load $(rospack find "
                "point_cloud_odometry)/config/parameters.yaml");
    // TODO: maybe is there a way to load this from config file?
    ros::param::set("frame_id/fixed", "test_fixed_frame");
    ros::param::set("frame_id/odometry", "test_odometry_frame");
    ros::param::set("fiducial_calibration/position/x", 0.5);
    ros::param::set("fiducial_calibration/position/y", 0.5);
    ros::param::set("fiducial_calibration/position/z", 0.5);
    ros::param::set("fiducial_calibration/orientation/x", 0.0);
    ros::param::set("fiducial_calibration/orientation/y", 0.0);
    ros::param::set("fiducial_calibration/orientation/z", 0.0);
    ros::param::set("fiducial_calibration/orientation/w", 1.0);
    ros::param::set("b_is_flat_ground_assumption", false);
    ros::param::set("b_verbose", true);
    // TODO: make sure that we don't want to load this from config file
    ros::param::set("icp/num_threads", 2);

    for (size_t i = 0; i < n_points_; i++) {
      data->at(i).x = 1024 * rand() / (RAND_MAX + 1.0f);
      data->at(i).y = 1024 * rand() / (RAND_MAX + 1.0f);
      data->at(i).z = 1024 * rand() / (RAND_MAX + 1.0f);
    }
  }

protected:
  virtual void SetUp() {}

  virtual void TearDown() {}

  void SetHealthyStatus(bool status) {
    pco.is_healthy_ = status;
  }

  bool LoadParameters(ros::NodeHandle& nh) {
    return pco.LoadParameters(nh);
  }
  bool IsImuIntegrationUsed() {
    return pco.b_use_imu_integration_;
  }
  bool IsOdometryIntegrationUsed() {
    return pco.b_use_odometry_integration_;
  }
  bool IsPoseStampedIntegrationUsed() {
    return pco.b_use_pose_stamped_integration_;
  }
  bool isGroundFlat() {
    return pco.b_is_flat_ground_assumption_;
  }
  PointCloudF GetPoints() {
    return pco.points_;
  }
  Eigen::Matrix3d GetImuDelta() {
    return pco.imu_delta_;
  }
  tf::Transform GetOdometryDelta() {
    return pco.odometry_delta_;
  }
  tf::Transform GetPoseStampedDelta() {
    return pco.pose_stamped_delta_;
  }
  ros::Time GetStamp() {
    return pco.stamp_;
  }

  pcl::Registration<PointF, PointF>::Ptr GetICP() {
    return pco.icp_;
  }
};

TEST_F(PointCloudOdometryTest, TestInitialize) {
  ros::NodeHandle nh;
  bool result = pco.Initialize(nh);
  ASSERT_TRUE(result);
}

TEST_F(PointCloudOdometryTest, TestLoadParams) {
  ros::NodeHandle nh;
  ASSERT_TRUE(LoadParameters(nh));
}

TEST_F(PointCloudOdometryTest, EnableDisableIMUIntegration) {
  ASSERT_FALSE(IsImuIntegrationUsed());
  pco.EnableImuIntegration();
  ASSERT_TRUE(IsImuIntegrationUsed());
  pco.EnableImuIntegration();
  ASSERT_TRUE(IsImuIntegrationUsed());
  pco.DisableImuIntegration();
  ASSERT_FALSE(IsImuIntegrationUsed());
}

TEST_F(PointCloudOdometryTest, EnableDisableOdometryIntegration) {
  ASSERT_FALSE(IsOdometryIntegrationUsed());
  pco.EnableOdometryIntegration();
  ASSERT_TRUE(IsOdometryIntegrationUsed());
  pco.EnableOdometryIntegration();
  ASSERT_TRUE(IsOdometryIntegrationUsed());
  pco.DisableOdometryIntegration();
  ASSERT_FALSE(IsOdometryIntegrationUsed());
}

TEST_F(PointCloudOdometryTest, EnableDisablePoseStampedIntegration) {
  ASSERT_FALSE(IsPoseStampedIntegrationUsed());
  pco.EnablePoseStampedIntegration();
  ASSERT_TRUE(IsPoseStampedIntegrationUsed());
  pco.EnablePoseStampedIntegration();
  ASSERT_TRUE(IsPoseStampedIntegrationUsed());
  pco.DisablePoseStampedIntegration();
  ASSERT_FALSE(IsPoseStampedIntegrationUsed());
}

TEST_F(PointCloudOdometryTest, SetLidar) {
  ASSERT_TRUE(pco.SetLidar(*data));
  EXPECT_EQ(GetPoints().size(), n_points_);
}
TEST_F(PointCloudOdometryTest, SetImuDelta) {
  Eigen::Matrix3d imu = Eigen::Matrix3d::Random();
  ASSERT_TRUE(pco.SetImuDelta(imu));
  ASSERT_TRUE(imu.isApprox(GetImuDelta()));
}

TEST_F(PointCloudOdometryTest, SetOdometryDelta) {
  tf::Transform odom = tf::Transform::getIdentity();
  odom.setOrigin(tf::Vector3(1.0, 5.0, 10.0));
  ASSERT_TRUE(pco.SetOdometryDelta(odom));
  EXPECT_DOUBLE_EQ(GetOdometryDelta().getOrigin().getX(), 1.0);
  EXPECT_DOUBLE_EQ(GetOdometryDelta().getOrigin().getY(), 5.0);
  EXPECT_DOUBLE_EQ(GetOdometryDelta().getOrigin().getZ(), 10.0);
}

TEST_F(PointCloudOdometryTest, SetPoseStampedDelta) {
  tf::Transform odom = tf::Transform::getIdentity();
  odom.setOrigin(tf::Vector3(1.0, 5.0, 10.0));
  ASSERT_TRUE(pco.SetPoseStampedDelta(odom));
  EXPECT_DOUBLE_EQ(GetPoseStampedDelta().getOrigin().getX(), 1.0);
  EXPECT_DOUBLE_EQ(GetPoseStampedDelta().getOrigin().getY(), 5.0);
  EXPECT_DOUBLE_EQ(GetPoseStampedDelta().getOrigin().getZ(), 10.0);
}

TEST_F(PointCloudOdometryTest, GetDiagnostics) {
  ros::NodeHandle nh;
  bool result = pco.Initialize(nh);
  ASSERT_TRUE(result);
  auto diagnostic = pco.GetDiagnostics();
  EXPECT_EQ(diagnostic.level, 2);
  EXPECT_TRUE(diagnostic.name == "/PointCloudOdometry");
  SetHealthyStatus(true);
  diagnostic = pco.GetDiagnostics();
  EXPECT_EQ(diagnostic.level, 0);
  EXPECT_TRUE(diagnostic.name == "/PointCloudOdometry");
}

TEST_F(PointCloudOdometryTest, GetLastPointCloud) {
  PointCloudF::Ptr pc(new PointCloudF);
  EXPECT_FALSE(pco.GetLastPointCloud(pc));
  pco.UpdateEstimate();
  EXPECT_TRUE(pco.GetLastPointCloud(pc));
}
TEST_F(PointCloudOdometryTest, SetFlatGroundAssumptionValue) {
  EXPECT_FALSE(isGroundFlat());
  pco.SetFlatGroundAssumptionValue(true);
  EXPECT_TRUE(isGroundFlat());
  pco.SetFlatGroundAssumptionValue(false);
  EXPECT_FALSE(isGroundFlat());
}

TEST_F(PointCloudOdometryTest, UpdateEstimateUpdateICP) {
  auto pc_box = GenerateHollowCubic(10, 10, 10, 0.1, 0.1, 0.1);
  PointCloudF::Ptr translated_pc_box(new PointCloudF());
  translated_pc_box->resize(pc_box->size());
  float offset = 0.05f;
  for (size_t i = 0; i < pc_box->points.size(); i++) {
    translated_pc_box->at(i).x = pc_box->at(i).x + offset;
    translated_pc_box->at(i).y = pc_box->at(i).y + offset;
    translated_pc_box->at(i).z = pc_box->at(i).z + 0;
  }
  ros::NodeHandle nh;

  EXPECT_TRUE(pco.Initialize(nh));
  EXPECT_TRUE(pco.SetLidar(*pc_box));
  EXPECT_FALSE(pco.UpdateEstimate());
  EXPECT_TRUE(pco.SetLidar(*translated_pc_box));
  EXPECT_TRUE(pco.UpdateEstimate());
  ASSERT_EQ(GetICP()->hasConverged(), true);
  EXPECT_LT(GetICP()->getFitnessScore(), 0.1);
  EXPECT_NEAR(
      GetICP()->getFinalTransformation().inverse()(0, 3), offset, epsiliond);
  EXPECT_NEAR(
      GetICP()->getFinalTransformation().inverse()(1, 3), offset, epsiliond);
  EXPECT_NEAR(
      GetICP()->getFinalTransformation().inverse()(2, 3), 0.0f, epsiliond);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudOdometryTest");
  return RUN_ALL_TESTS();
}
