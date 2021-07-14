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
#include <frontend_utils/CommonStructs.h>
#include <pcl/io/pcd_io.h>

const double epsilion = 1e-4;

PointCloudF::Ptr GeneratePlane(size_t x_points = 10,
                               size_t y_points = 10,
                               float step_x = 0.1f,
                               float step_y = 0.1f,
                               std::string frame_name = "dummy") {
  auto pc_out = boost::make_shared<PointCloudF>();
  pc_out->reserve(x_points * y_points);
  pc_out->header.frame_id = frame_name;

  for (size_t ix = 0; ix < x_points; ix++) {
    for (size_t iy = 0; iy < y_points; iy++) {
      pc_out->push_back(
          PointF({ix * step_x, iy * step_y, 0.0, 0.0, 0.0, 0.0, 1.0}));
    }
  }

  return pc_out;
}

TEST(CreatePlaneTest, GeneratePlane) {
  auto cubic = GeneratePlane();
  EXPECT_EQ(cubic->points.size(), 100);
}

PointCloudF::Ptr GeneratePlaneXYZ(size_t x_points = 10,
                                  size_t y_points = 10,
                                  float step_x = 0.1f,
                                  float step_y = 0.1f,
                                  std::string frame_name = "dummy") {
  auto pc_out = boost::make_shared<PointCloudF>();
  pc_out->reserve(x_points * y_points);
  pc_out->header.frame_id = frame_name;

  for (size_t ix = 0; ix < x_points; ix++) {
    for (size_t iy = 0; iy < y_points; iy++) {
      pc_out->push_back(
          PointF({ix * step_x, iy * step_y, 0.0, 0.0, 0.0, 0.0, 1.0}));
    }
  }

  return pc_out;
}

TEST(CreatePlaneTest, GeneratePlaneXYZ) {
  auto cubic = GeneratePlaneXYZ();
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
      const PointCloudF::Ptr pcl_normalized,
      const PointCloudLocalization::PointNormal::Ptr pcl_normals,
      const std::vector<size_t>& correspondences,
      const Eigen::Matrix4f& T,
      Eigen::Matrix<double, 6, 6>& Ap) {
    point_cloud_localization.ComputeAp_ForPoint2PlaneICP(
        pcl_normalized, pcl_normals, correspondences, T, Ap);
  }

  void computeAp_ForPoint2PlaneICP(const PointCloudF::Ptr pcl_normalized,
                                   const PointCloudF& pcl_normals,
                                   const std::vector<size_t>& correspondences,
                                   const Eigen::Matrix4f& T,
                                   Eigen::Matrix<double, 6, 6>& Ap) {
    point_cloud_localization.ComputeAp_ForPoint2PlaneICP(
        pcl_normalized, pcl_normals, correspondences, T, Ap);
  }

  void computeIcpObservability(const PointCloudF& query_cloud,
                               const PointCloudF& reference_cloud,
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
  PointCloudF transformed_cloud; //(new PointCloudF);
  point_cloud_localization.TransformPointsToFixedFrame(*dummy_cloud,
                                                       &transformed_cloud);

  for (const auto& point : transformed_cloud.points) {
    EXPECT_NEAR(point._PointXYZINormal::z, 3.0, epsilion);
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

  PointCloudF transformed_cloud; //(new PointCloudF);
  point_cloud_localization.TransformPointsToSensorFrame(*dummy_cloud,
                                                        &transformed_cloud);

  for (const auto& point : transformed_cloud.points) {
    EXPECT_NEAR(point._PointXYZINormal::z, -3.0, epsilion);
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
TEST_F(PointCloudLocalizationTest, ComputeAp_ForPoint2PlaneICP) {
  auto dummy_cloud = GeneratePlane();
  // With new normal computation prior to point_cloud_localization, we don't
  // need the lines below
  PointCloudLocalization::PointNormal::Ptr dummy_normals(
      new PointCloudLocalization::PointNormal);
  Eigen::Matrix<double, 6, 6> Ap = Eigen::Matrix<double, 6, 6>::Zero();
  PointCloudF::Ptr dummy_normalized(new PointCloudF);
  addNormal(*dummy_cloud, dummy_normals, 10);
  normalizePCloud(*dummy_cloud, dummy_normalized);
  std::vector<size_t> correspondences(dummy_cloud->size());
  std::iota(std::begin(correspondences),
            std::end(correspondences),
            0);  // Fill with 0, 1, ...
  // Generate Ap manually
  Eigen::MatrixXf Ap_ref = Eigen::MatrixXf::Zero(6, 6);
  Eigen::Vector3d a_i, n_i;
  for (size_t i = 0; i < dummy_cloud->size(); i++) {
    a_i << dummy_normalized->points[i].x,  //////
        dummy_normalized->points[i].y,     //////
        dummy_normalized->points[i].z;

    n_i << dummy_normals->points[correspondences[i]].normal_x,  //////
        dummy_normals->points[correspondences[i]].normal_y,     //////
        dummy_normals->points[correspondences[i]].normal_z;
    double a, b, c;
    a = (a_i.cross(n_i))(0);
    b = (a_i.cross(n_i))(1);
    c = n_i(2);
    Ap_ref(0, 0) += a * a;
    Ap_ref(0, 1) += a * b;
    Ap_ref(1, 0) += a * b;
    Ap_ref(1, 1) += b * b;
    Ap_ref(0, 5) += a * c;
    Ap_ref(1, 5) += b * c;
    Ap_ref(5, 0) += a * c;
    Ap_ref(5, 1) += b * c;
    Ap_ref(5, 5) += c * c;
  }

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  computeAp_ForPoint2PlaneICP(
      dummy_normalized, dummy_normals, correspondences, T, Ap);
  
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      EXPECT_NEAR(Ap(i, j), Ap_ref(i, j), epsilion);
    }
  }
  EXPECT_NEAR(Ap(0, 0), 56.7753, epsilion);
  EXPECT_NEAR(Ap(1, 1), 56.7753, epsilion);
  EXPECT_NEAR(Ap(5, 5), 100, epsilion);
}

TEST_F(PointCloudLocalizationTest, ComputeAp_ForPoint2PlaneICP_XYZINORMAL) {
  auto dummy_cloud = GeneratePlane();
  // With new normal computation prior to point_cloud_localization, we don't
  // need the lines below
  PointCloudLocalization::PointNormal::Ptr dummy_normals(
      new PointCloudLocalization::PointNormal);
  Eigen::Matrix<double, 6, 6> Ap = Eigen::Matrix<double, 6, 6>::Zero();
  PointCloudF::Ptr dummy_normalized(new PointCloudF);
  addNormal(*dummy_cloud, dummy_cloud, 10);
  normalizePCloud(*dummy_cloud, dummy_normalized);
  std::vector<size_t> correspondences(dummy_cloud->size());
  std::iota(std::begin(correspondences),
            std::end(correspondences),
            0); // Fill with 0, 1, ...
  // Generate Ap manually
  Eigen::MatrixXf Ap_ref = Eigen::MatrixXf::Zero(6, 6);
  Eigen::Vector3d a_i, n_i;
  for (size_t i = 0; i < dummy_cloud->size(); i++) {
    a_i << dummy_normalized->points[i].x, //////
        dummy_normalized->points[i].y,    //////
        dummy_normalized->points[i].z;

    n_i << dummy_normalized->points[correspondences[i]].normal_x, //////
        dummy_normalized->points[correspondences[i]].normal_y,    //////
        dummy_normalized->points[correspondences[i]].normal_z;
    double a, b, c;
    a = (a_i.cross(n_i))(0);
    b = (a_i.cross(n_i))(1);
    c = n_i(2);
    Ap_ref(0, 0) += a * a;
    Ap_ref(0, 1) += a * b;
    Ap_ref(1, 0) += a * b;
    Ap_ref(1, 1) += b * b;
    Ap_ref(0, 5) += a * c;
    Ap_ref(1, 5) += b * c;
    Ap_ref(5, 0) += a * c;
    Ap_ref(5, 1) += b * c;
    Ap_ref(5, 5) += c * c;
  }

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  computeAp_ForPoint2PlaneICP(
      dummy_normalized, *dummy_normalized, correspondences, T, Ap);

  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      EXPECT_NEAR(Ap(i, j), Ap_ref(i, j), epsilion);
    }
  }
  EXPECT_NEAR(Ap(0, 0), 56.7753, epsilion);
  EXPECT_NEAR(Ap(1, 1), 56.7753, epsilion);
  EXPECT_NEAR(Ap(5, 5), 100, epsilion);
}

// TODO: since i don't have a reference, i just outputed the result and i'm
// checking consistency assuming that the implementation was correct.
TEST_F(PointCloudLocalizationTest, ComputePoint2PlaneICPCovariance) {
  auto dummy_cloud = GeneratePlane();
  addNormal(*dummy_cloud, dummy_cloud, 10);
  PointCloudF::Ptr dummy_normalized(new PointCloudF);
  normalizePCloud(*dummy_cloud, dummy_normalized);
  Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();
  std::vector<size_t> correspondences(dummy_normalized->size());
  std::iota(std::begin(correspondences),
            std::end(correspondences),
            0);  // Fill with 0, 1, ...
  point_cloud_localization.ComputePoint2PlaneICPCovariance(
      *dummy_normalized, *dummy_normalized, correspondences, tf, &cov);

  // Default to max value since single plane not "observable"
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = 0; j < 6; j++) {
      if (i == j) {
        EXPECT_NEAR(cov(i, j), 0.01, epsilion);
      } else {
        EXPECT_NEAR(cov(i, j), 0.0, epsilion);
      }
    }
  }

  // TODO FIX THIS BELOW!!
  // // Add another plane
  // // Perform transformation
  // PointCloudF::Ptr transformed_dummy(new PointCloudF());
  // Eigen::Matrix4f T_dummy = Eigen::Matrix4f::Zero();
  // T_dummy(0, 0) = 1;
  // T_dummy(1, 2) = -1;
  // T_dummy(2, 1) = 1;
  // T_dummy(0, 3) = 1;
  // pcl::transformPointCloudWithNormals(*dummy_normalized, *transformed_dummy,
  // T_dummy,true);
  // // addNormal(*transformed_dummy, transformed_dummy, 10);
  // *dummy_normalized += *transformed_dummy;

  // cov = Eigen::Matrix<double, 6, 6>::Zero();
  // tf = Eigen::Matrix4f::Identity();
  // std::vector<size_t> correspondences2(dummy_normalized->size());
  // std::iota(std::begin(correspondences2),
  //           std::end(correspondences2),
  //           0);  // Fill with 0, 1, ...
  // point_cloud_localization.ComputePoint2PlaneICPCovariance(
  //     *dummy_normalized, *dummy_normalized, correspondences2, tf, &cov);

  // pcl::io::savePCDFileASCII ("/home/bmorrell/test2.pcd", *dummy_normalized);
  // // Default to max value since single plane not "observable"
  // for (size_t i = 0; i < 6; i++) {
  //   for (size_t j = 0; j < 6; j++) {
  //     if (i == j) {
  //       EXPECT_NEAR(cov(i, j), 0.0011, epsilion);
  //     } else {
  //       EXPECT_NEAR(cov(i, j), 0.0, epsilion);
  //     }
  //   }
  // }
}
// TODO: since i don't have a reference, i just outputed the result and i'm
// checking consistency assuming that the implementation was correct.
TEST_F(PointCloudLocalizationTest, ComputeIcpObservability) {
  // TODO: known issue: sometimes fail

  Eigen::Matrix<double, 6, 6> eigenvectors_new =
      Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> eigenvalues_new =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 6> observability_matrix =
      Eigen::Matrix<double, 6, 6>::Zero();
  auto query = GeneratePlane();
  std::vector<size_t> correspondences(query->size());
  std::iota(std::begin(correspondences),
            std::end(correspondences),
            0);  // Fill with 0, 1, ...
  computeIcpObservability(*query,
                          *query,
                          correspondences,
                          Eigen::Matrix4f::Identity(),
                          &eigenvectors_new,
                          &eigenvalues_new,
                          &observability_matrix);
  // Note Eigen sorts this automatically 
  EXPECT_NEAR(eigenvalues_new(0), 0, epsilion);
  EXPECT_NEAR(eigenvalues_new(1), 0, epsilion);
  EXPECT_NEAR(eigenvalues_new(2), 0, epsilion);
  EXPECT_NEAR(eigenvalues_new(3), 56.7753, epsilion);
  EXPECT_NEAR(eigenvalues_new(4), 56.7753, epsilion);
  EXPECT_NEAR(eigenvalues_new(5), 100, epsilion);
}

TEST_F(PointCloudLocalizationTest, MeasurementUpdateGetDiagnostics) {
  ros::NodeHandle nh;
  bool result = point_cloud_localization.Initialize(nh);
  ASSERT_TRUE(result);
  auto diagnostic = point_cloud_localization.GetDiagnostics();
  EXPECT_EQ(diagnostic.level, 2);
  EXPECT_TRUE(diagnostic.name == "/PointCloudLocalization");
  PointCloudF::Ptr dummy1, dummy2;
  point_cloud_localization.MeasurementUpdate(dummy1, dummy2, nullptr);
  diagnostic = point_cloud_localization.GetDiagnostics();
  EXPECT_EQ(diagnostic.level, 2);
  EXPECT_TRUE(diagnostic.name == "/PointCloudLocalization");
  PointCloudF::Ptr dummy3(new PointCloudF), dummy4(new PointCloudF);
  PointCloudF dummy5;
  dummy3 = GeneratePlane();
  dummy4 = GeneratePlane();
  point_cloud_localization.MeasurementUpdate(dummy3, dummy4, &dummy5);
  diagnostic = point_cloud_localization.GetDiagnostics();
  EXPECT_EQ(diagnostic.level, 0);
}

// todo in general this test should be deleted since addnormal should be deleted
TEST_F(PointCloudLocalizationTest, addNormalTest) {
  PointCloudLocalization::PointNormal::Ptr pcl_normals(
      new PointCloudLocalization::PointNormal);
  auto point_cloud = GeneratePlaneXYZ(1000, 1000);
  addNormal(*point_cloud, pcl_normals, 10);
  EXPECT_EQ(pcl_normals->size(), 1000 * 1000);
}

TEST_F(PointCloudLocalizationTest, normalizePCloud) {
  PointCloudF::Ptr new_cloud = GeneratePlane(1000, 1000);
  PointCloudF::Ptr new_normalized(new PointCloudF);

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
