#include <chrono>
#include <frontend_utils/CommonFunctions.h>
#include <gtest/gtest.h>
#include <omp.h>

const double epsiliond = 0.00000001;
const double epsilionF = 0.00000001f;

PointF GetPointWithNormalInZ() {
  PointF point_with_normal;
  point_with_normal.x = 1.0;
  point_with_normal.y = 2.0;
  point_with_normal.z = 3.0;
  point_with_normal.normal_x = 0.0;
  point_with_normal.normal_y = 0.0;
  point_with_normal.normal_z = 1.0;
  return point_with_normal;
}

PointCloudF::Ptr GeneratePointCloudInZ(int N) {
  PointCloudF::Ptr out(new PointCloudF);
  for (size_t i = 0; i < N; ++i) {
    out->push_back(GetPointWithNormalInZ());
  }
  return out;
}

TEST(PCLTwoPlaneVectorsFromNormal, working_double) {
  auto point_with_normal = GetPointWithNormalInZ();
  auto matrixd = PCLTwoPlaneVectorsFromNormal<double>(point_with_normal);
  EXPECT_NEAR(matrixd(0, 0), 1.0, epsiliond);
  EXPECT_NEAR(matrixd(1, 1), 1.0, epsiliond);
  EXPECT_NEAR(matrixd(2, 2), 0.001, epsiliond);
}

TEST(PCLTwoPlaneVectorsFromNormal, working_float) {
  auto point_with_normal = GetPointWithNormalInZ();
  Eigen::Matrix3f matrixf =
      PCLTwoPlaneVectorsFromNormal<float>(point_with_normal);
  EXPECT_NEAR(matrixf(0, 0), 1.0, epsiliond);
  EXPECT_NEAR(matrixf(1, 1), 1.0, epsiliond);
  EXPECT_NEAR(matrixf(2, 2), 0.001, epsiliond);
}

TEST(CalculateCovarianceFromNormals, working_double) {
  const PointCloudF::ConstPtr cloud =
      GeneratePointCloudInZ(1000000); //(new PointCloudF);
  MatricesVectorPtr cov = boost::make_shared<MatricesVector>();
  auto t_start = std::chrono::high_resolution_clock::now();
  CalculateCovarianceFromNormals(cloud, *cov, 1);
  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count() * 1e-3;
  std::cout << "elapsed time: " << elapsed_time_ms << " s " << std::endl;

  for (size_t i = 0; i < cov->size(); i++) {
    EXPECT_NEAR((*cov)[i](0, 0), 1.0, epsiliond);
    EXPECT_NEAR((*cov)[i](1, 1), 1.0, epsiliond);
    EXPECT_NEAR((*cov)[i](2, 2), 0.001, epsiliond);
  }
}

TEST(CalculateCovarianceFromNormals, working_float) {
  const PointCloudF::ConstPtr cloud = GeneratePointCloudInZ(1000000);
  MatricesVectorfPtr cov = boost::make_shared<MatricesVectorf>();
  auto t_start = std::chrono::high_resolution_clock::now();
  CalculateCovarianceFromNormals(cloud, *cov, 1);
  auto t_end = std::chrono::high_resolution_clock::now();
  double elapsed_time_ms =
      std::chrono::duration<double, std::milli>(t_end - t_start).count() * 1e-3;
  std::cout << "elapsed time: " << elapsed_time_ms << " s " << std::endl;

  for (size_t i = 0; i < cov->size(); i++) {
    EXPECT_NEAR((*cov)[i](0, 0), 1.0, epsiliond);
    EXPECT_NEAR((*cov)[i](1, 1), 1.0, epsiliond);
    EXPECT_NEAR((*cov)[i](2, 2), 0.001, epsiliond);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
