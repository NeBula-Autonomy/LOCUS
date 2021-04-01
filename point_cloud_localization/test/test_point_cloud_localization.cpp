/**
 *  @author Abhishek Thakur
 *  @author Andrzej Reinke
 *  @brief Test cases for point_cloud_localization class
 * The following functions don't have unitests:
 * PublishAll - ros specific
 * PublishPose - ros specific
 * PublishPoseNoUpdate - ros specific
 * PublishConditionNumber - ros specific
 * PublishObservableDirections - ros specific
 * PublishOdometry - ros specific
 * UpdateTimestamp - too simple
 * /////////////////////////////////////////////////////////////////////////
 *  TODO: decide whether GeneratePlane
 * tests should be here or in a test_common repo (that currently doesn't exist)
 */

#include <gtest/gtest.h>

#include "point_cloud_localization/PointCloudLocalization.h"
#include "point_cloud_localization/utils.h"

const double epsilion = 1e-4;

PointCloudLocalization::PointCloud::Ptr
GeneratePlane(size_t x_points = 10,
              size_t y_points = 10,
              float step_x = 0.1f,
              float step_y = 0.1f,
              std::string frame_name = "dummy") {
  auto pc_out = boost::make_shared<PointCloudLocalization::PointCloud>();
  pc_out->reserve(x_points * y_points);
  pc_out->header.frame_id = frame_name;

  for (size_t ix = 0; ix < x_points; ix++) {
    for (size_t iy = 0; iy < y_points; iy++) {
      pc_out->push_back(pcl::PointXYZI({ix * step_x, iy * step_y, 0.0, 0.0}));
    }
  }

  return pc_out;
}

TEST(CreatePlaneTest, GeneratePlane) {
  auto cubic = GeneratePlane();
  EXPECT_EQ(cubic->points.size(), 100);
}

class PointCloudLocalizationTest : public ::testing::Test {
public:
  PointCloudLocalizationTest() {
    std::system("rosparam load $(rospack find "
                "point_cloud_localization)/config/parameters.yaml");
    ros::param::set("frame_id/fixed", "test_fixed_frame");
    ros::param::set("frame_id/base", "test_odometry_frame");
    ros::param::set("fiducial_calibration/position/x", 0.5);
    ros::param::set("fiducial_calibration/position/y", 0.5);
    ros::param::set("fiducial_calibration/position/z", 0.5);
    ros::param::set("fiducial_calibration/orientation/x", 0.0);
    ros::param::set("fiducial_calibration/orientation/y", 0.0);
    ros::param::set("fiducial_calibration/orientation/z", 0.0);
    ros::param::set("fiducial_calibration/orientation/w", 1.0);
    ros::param::set("b_is_flat_ground_assumption", false);
    ros::param::set("localization/num_threads", 2);
  }
  PointCloudLocalization point_cloud_localization;
  bool isGroundFlat() {
    return point_cloud_localization.b_is_flat_ground_assumption_;
  }

  void computeAp_ForPoint2PlaneICP(
      const PointCloudLocalization::PointCloud::Ptr pcl_normalized,
      const PointCloudLocalization::PointNormal::Ptr pcl_normals,
      const std::vector<size_t>& correspondences,
      const Eigen::Matrix4f& T,
      Eigen::Matrix<double, 6, 6>& Ap) {
    point_cloud_localization.ComputeAp_ForPoint2PlaneICP(
        pcl_normalized, pcl_normals, correspondences, T, Ap);
  }

  void computeIcpObservability(
      const PointCloudLocalization::PointCloud& query_cloud,
      const PointCloudLocalization::PointCloud& reference_cloud,
      const std::vector<size_t>& correspondences,
      const Eigen::Matrix4f& T,
      Eigen::Matrix<double, 6, 6>* eigenvectors_ptr,
      Eigen::Matrix<double, 6, 1>* eigenvalues_ptr,
      Eigen::Matrix<double, 6, 6>* A_ptr) {
    point_cloud_localization.ComputeIcpObservability(query_cloud,
                                                     reference_cloud,
                                                     correspondences,
                                                     T,
                                                     eigenvectors_ptr,
                                                     eigenvalues_ptr,
                                                     A_ptr);
  }

protected:
  virtual void SetUp() {}

  virtual void TearDown() {}
};

TEST_F(PointCloudLocalizationTest, TestInitialize) {
  ros::NodeHandle nh;
  bool result = point_cloud_localization.Initialize(nh);
  ASSERT_TRUE(result);
}

TEST_F(PointCloudLocalizationTest, GetIncrementalEstimateInit) {
  ros::NodeHandle nh;
  ASSERT_TRUE(point_cloud_localization.Initialize(nh));
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.translation.X(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.translation.Y(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.translation.Z(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.rotation.Pitch(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.rotation.Roll(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.rotation.Yaw(),
              0.0,
              epsilion);
}

TEST_F(PointCloudLocalizationTest, GetGetIntegratedEstimateInit) {
  ros::NodeHandle nh;
  ASSERT_TRUE(point_cloud_localization.Initialize(nh));
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.translation.X(),
              0.5,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.translation.Y(),
              0.5,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.translation.Z(),
              0.5,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.rotation.Pitch(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.rotation.Roll(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.rotation.Yaw(),
              0.0,
              epsilion);
}

TEST_F(PointCloudLocalizationTest, MotionUpdate) {
  ros::NodeHandle nh;
  ASSERT_TRUE(point_cloud_localization.Initialize(nh));
  geometry_utils::Vector3Base<double> tr(1.0, 2.0, 3.0);
  geometry_utils::Rotation3Base<double> rot;
  geometry_utils::Transform3 tf(tr, rot);
  point_cloud_localization.MotionUpdate(tf);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.translation.X(),
              1.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.translation.Y(),
              2.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.translation.Z(),
              3.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.rotation.Pitch(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.rotation.Roll(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.incremental_estimate_.rotation.Yaw(),
              0.0,
              epsilion);
}

TEST_F(PointCloudLocalizationTest, SetIntegratedEstimate) {
  ros::NodeHandle nh;
  ASSERT_TRUE(point_cloud_localization.Initialize(nh));
  geometry_utils::Vector3Base<double> tr(2.0, 2.0, 3.0);
  geometry_utils::Rotation3Base<double> rot;
  geometry_utils::Transform3 tf(tr, rot);
  point_cloud_localization.SetIntegratedEstimate(tf);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.translation.X(),
              2.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.translation.Y(),
              2.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.translation.Z(),
              3.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.rotation.Pitch(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.rotation.Roll(),
              0.0,
              epsilion);
  EXPECT_NEAR(point_cloud_localization.integrated_estimate_.rotation.Yaw(),
              0.0,
              epsilion);
}
// tests whether dummy plane point cloud was moved 3m up
TEST_F(PointCloudLocalizationTest, TransformPointsToFixedFrame) {
  ros::NodeHandle nh;
  ASSERT_TRUE(point_cloud_localization.Initialize(nh));
  geometry_utils::Vector3Base<double> tr(2.0, 2.0, 3.0);
  geometry_utils::Rotation3Base<double> rot;
  geometry_utils::Transform3 tf(tr, rot);
  point_cloud_localization.SetIntegratedEstimate(tf);
  auto dummy_cloud = GeneratePlane();
  PointCloudLocalization::PointCloud
      transformed_cloud; //(new PointCloudLocalization::PointCloud);
  point_cloud_localization.TransformPointsToFixedFrame(*dummy_cloud,
                                                       &transformed_cloud);

  for (const auto& point : transformed_cloud.points) {
    EXPECT_NEAR(point._PointXYZI::z, 3.0, epsilion);
  }
}
// tests whether dummy plane point cloud was moved 3m up
TEST_F(PointCloudLocalizationTest, TransformPointsToSensorFrame) {
  ros::NodeHandle nh;
  ASSERT_TRUE(point_cloud_localization.Initialize(nh));
  geometry_utils::Vector3Base<double> tr(2.0, 2.0, 3.0);
  geometry_utils::Rotation3Base<double> rot;
  geometry_utils::Transform3 tf(tr, rot);
  point_cloud_localization.SetIntegratedEstimate(tf);
  auto dummy_cloud = GeneratePlane();

  PointCloudLocalization::PointCloud
      transformed_cloud; //(new PointCloudLocalization::PointCloud);
  point_cloud_localization.TransformPointsToSensorFrame(*dummy_cloud,
                                                        &transformed_cloud);

  for (const auto& point : transformed_cloud.points) {
    EXPECT_NEAR(point._PointXYZI::z, -3.0, epsilion);
  }
}

TEST_F(PointCloudLocalizationTest, SetFlatGroundAssumptionValue) {
  EXPECT_FALSE(isGroundFlat());
  point_cloud_localization.SetFlatGroundAssumptionValue(true);
  EXPECT_TRUE(isGroundFlat());
  point_cloud_localization.SetFlatGroundAssumptionValue(false);
  EXPECT_FALSE(isGroundFlat());
}

// TODO: since i don't have a reference, i just outputed the result and i'm
// checking consistency assuming that the implementation was correct.
TEST_F(PointCloudLocalizationTest, ComputePoint2PointICPCovariance) {
  auto dummy_cloud = GeneratePlane();
  Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();

  point_cloud_localization.ComputePoint2PointICPCovariance(
      *dummy_cloud, tf, &cov);

  EXPECT_NEAR(cov.diagonal()[0], 1000, epsilion);
  EXPECT_NEAR(cov.diagonal()[1], 1000, epsilion);
  EXPECT_NEAR(cov.diagonal()[2], 1000, epsilion);
  EXPECT_NEAR(cov.diagonal()[3], 1000, epsilion);
  EXPECT_NEAR(cov.diagonal()[4], 1000, epsilion);
  EXPECT_NEAR(cov.diagonal()[5], 1000, epsilion);
}
// TODO: since i don't have a reference, i just outputed the result and i'm
// checking consistency assuming that the implementation was correct.
// TEST_F(PointCloudLocalizationTest, ComputeAp_ForPoint2PlaneICP) {
//   auto dummy_cloud = GeneratePlane();
//   PointCloudLocalization::PointNormal::Ptr dummy_normals(
//       new PointCloudLocalization::PointNormal);
//   Eigen::Matrix<double, 6, 6> Ap = Eigen::Matrix<double, 6, 6>::Zero();
//   PointCloudLocalization::PointCloud::Ptr dummy_normalized(
//       new PointCloudLocalization::PointCloud);
//   addNormal(*dummy_cloud, dummy_normals, 10);
//   normalizePCloud(*dummy_cloud, dummy_normalized);
//   computeAp_ForPoint2PlaneICP(dummy_normalized, dummy_normals, Ap);

//   for (size_t i = 0; i < 6; i++) {
//     for (size_t j = 0; j < 6; j++) {
//       // since the plane is generated the diagonal cov will not be 0 only in
//       // x:(0), y:(1), yaw:(5)
//       if (not(i == j and (i == 0 or i == 1 or i == 5)))
//         EXPECT_NEAR(Ap(i, j), 0.0, epsilion);
//     }
//   }
//   EXPECT_NEAR(Ap(0, 0), 56.7753, epsilion);
//   EXPECT_NEAR(Ap(1, 1), 56.7753, epsilion);
//   EXPECT_NEAR(Ap(5, 5), 100, epsilion);
// }
// // TODO: since i don't have a reference, i just outputed the result and i'm
// // checking consistency assuming that the implementation was correct.
// TEST_F(PointCloudLocalizationTest, ComputePoint2PlaneICPCovariance) {
//   auto dummy_cloud = GeneratePlane();
//   Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
//   Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();

//   point_cloud_localization.ComputePoint2PlaneICPCovariance(
//       *dummy_cloud, tf, &cov);

//   ROS_INFO_STREAM(cov);
//   std::cout << cov << std::endl;
//   for (size_t i = 0; i < 6; i++) {
//     for (size_t j = 0; j < 6; j++) {
//       // since the plane is generated the diagonal cov will not be 0 only in
//       // x:(0), y:(1), yaw:(5)
//       if (not(i == j and (i == 0 or i == 1 or i == 5)))
//         EXPECT_NEAR(cov(i, j), 0.0, epsilion);
//     }
//   }
//   EXPECT_NEAR(cov(0, 0), 0.0166334, epsilion);
//   EXPECT_NEAR(cov(1, 1), 0.0166334, epsilion);
//   EXPECT_NEAR(cov(5, 5), 0.0292969, epsilion);
// }
// // TODO: since i don't have a reference, i just outputed the result and i'm
// // checking consistency assuming that the implementation was correct.
// TEST_F(PointCloudLocalizationTest, ComputeIcpObservability) {
//   // TODO: known issue: sometimes fail

//   Eigen::Matrix<double, 6, 6> eigenvectors_new =
//       Eigen::Matrix<double, 6, 6>::Zero();
//   Eigen::Matrix<double, 6, 1> eigenvalues_new =
//       Eigen::Matrix<double, 6, 1>::Zero();
//   Eigen::Matrix<double, 6, 6> observability_matrix =
//       Eigen::Matrix<double, 6, 6>::Zero();
//   auto query = GeneratePlane();
//   computeIcpObservability(
//       query, query, &eigenvectors_new, &eigenvalues_new,
//       &observability_matrix);
//   for (size_t i = 0; i < 6; i++) {
//     for (size_t j = 0; j < 6; j++) {
//       // since the plane is generated the diagonal cov will not be 0 only in
//       // x:(0), y:(1), yaw:(5)
//       if (not(i == j and (i == 0 or i == 1 or i == 5)))
//         EXPECT_NEAR(observability_matrix(i, j), 0.0, epsilion);
//     }
//   }
//   EXPECT_NEAR(observability_matrix(0, 0), 56.7753, epsilion);
//   EXPECT_NEAR(observability_matrix(1, 1), 56.7753, epsilion);
//   EXPECT_NEAR(observability_matrix(5, 5), 100, epsilion);
// }
TEST_F(PointCloudLocalizationTest, MeasurementUpdateGetDiagnostics) {
  ROS_INFO_STREAM("XD");
  ros::NodeHandle nh;
  bool result = point_cloud_localization.Initialize(nh);
  ASSERT_TRUE(result);
  auto diagnostic = point_cloud_localization.GetDiagnostics();
  EXPECT_EQ(diagnostic.level, 2);
  EXPECT_TRUE(diagnostic.name == "/PointCloudLocalization");
  PointCloudLocalization::PointCloud::Ptr dummy1, dummy2;
  point_cloud_localization.MeasurementUpdate(dummy1, dummy2, nullptr);
  diagnostic = point_cloud_localization.GetDiagnostics();
  EXPECT_EQ(diagnostic.level, 2);
  EXPECT_TRUE(diagnostic.name == "/PointCloudLocalization");
  PointCloudLocalization::PointCloud::Ptr dummy3(
      new PointCloudLocalization::PointCloud),
      dummy4(new PointCloudLocalization::PointCloud);
  PointCloudLocalization::PointCloud dummy5;
  dummy3 = GeneratePlane();
  dummy4 = GeneratePlane();
  point_cloud_localization.MeasurementUpdate(dummy3, dummy4, &dummy5);
  diagnostic = point_cloud_localization.GetDiagnostics();
  EXPECT_EQ(diagnostic.level, 0);
}

TEST_F(PointCloudLocalizationTest, addNormalTest) {
  PointCloudLocalization::PointNormal::Ptr pcl_normals(
      new PointCloudLocalization::PointNormal);
  PointCloudLocalization::PointCloud::Ptr point_cloud =
      GeneratePlane(1000, 1000);
  addNormal(*point_cloud, pcl_normals, 10);
  EXPECT_EQ(pcl_normals->size(), 1000 * 1000);
}

TEST_F(PointCloudLocalizationTest, normalizePCloud) {
  PointCloudLocalization::PointCloud::Ptr new_cloud = GeneratePlane(1000, 1000);
  PointCloudLocalization::PointCloud::Ptr new_normalized(
      new PointCloudLocalization::PointCloud);

  normalizePCloud(*new_cloud, new_normalized);

  EXPECT_EQ(new_normalized->size(), 1000 * 1000);
}

TEST_F(PointCloudLocalizationTest, doEigenDecomp6x6) {}

TEST_F(PointCloudLocalizationTest, doEigenDecomp3x3) {
  Eigen::Matrix<double, 3, 3> data;
  data << -2.0, -4.0, 2.0, -2.0, 1.0, 2.0, 4.0, 2.0, 5.0;

  Eigen::Matrix<double, 3, 1> eigenvalues;
  Eigen::Matrix<double, 3, 3> eigenvectors;
  doEigenDecomp3x3(data, eigenvalues, eigenvectors);

  EXPECT_NEAR(eigenvalues(0), -5.0, epsilion);
  EXPECT_NEAR(eigenvalues(1), 2.0, epsilion);
  EXPECT_NEAR(eigenvalues(2), 7.0, epsilion);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "PointCloudLocalizationTest");
  return RUN_ALL_TESTS();
}
