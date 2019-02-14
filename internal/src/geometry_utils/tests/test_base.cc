#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_base
#include <boost/test/unit_test.hpp>

#include <ros/ros.h>
#include <geometry_utils/GeometryUtils.h>

namespace gu = geometry_utils;

BOOST_AUTO_TEST_CASE(vector2_base) {
  Eigen::Vector2d p1 = Eigen::Vector2d::Random();
  Eigen::Vector2d p2 = Eigen::Vector2d::Random();

  gu::Vec2 v1(p1);
  gu::Vec2 v2(p2);

  Eigen::Vector2d e1 = v1.Eigen();
  Eigen::Vector2d e2 = v2.Eigen();
  BOOST_CHECK_EQUAL(gu::Vec2(e1 + e2), (v1 + v2));

  BOOST_CHECK_EQUAL(gu::Vec2(p1 + p2), (v1 + v2));
  BOOST_CHECK_EQUAL(gu::Vec2(p1 - p2), (v1 - v2));
  BOOST_CHECK_EQUAL(gu::Vec2(p1.cwiseProduct(p2)), (v1 % v2));
  BOOST_CHECK_EQUAL(gu::Vec2(p1.cwiseQuotient(p2)), (v1 / v2));
  double s = Eigen::MatrixXd::Random(1,1)(0);
  BOOST_CHECK_EQUAL(gu::Vec2(s*p1), (s*v1));
  BOOST_CHECK_EQUAL(gu::Vec2(p1*s), (v1*s));

  gu::Vec2 v3(0.1, 0.2);
  Eigen::Vector2d p3 = v3.Eigen();
  Eigen::Vector2d p4 = gu::Eigen(v3);
  BOOST_CHECK_EQUAL(gu::Vec2(p3), gu::Vec2(p4));
  BOOST_CHECK_EQUAL(gu::Vec2(s*p3), (s*v3));
  BOOST_CHECK_EQUAL(gu::Vec2(p1/p1.norm()), v1.Normalize());

  v3 /= s;
  p3 /= s;
  BOOST_CHECK_EQUAL(v3, gu::Vec2(p3));

  v3 += v2;
  p3 += p2;
  BOOST_CHECK_EQUAL(v3, gu::Vec2(p3));

  v3 -= v2;
  p3 -= p2;
  BOOST_CHECK_EQUAL(v3, gu::Vec2(p3));

  v3 %= v2;
  p3 = p3.cwiseProduct(p2);
  BOOST_CHECK_EQUAL(v3, gu::Vec2(p3));

  v3 /= v2;
  p3 = p3.cwiseQuotient(p2);
  BOOST_CHECK_EQUAL(v3, gu::Vec2(p3));
}

BOOST_AUTO_TEST_CASE(vector3_base) {
  Eigen::Vector3d p1 = Eigen::Vector3d::Random();
  Eigen::Vector3d p2 = Eigen::Vector3d::Random();

  gu::Vec3 v1(p1);
  gu::Vec3 v2(p2);

  Eigen::Vector3d e1 = v1.Eigen();
  Eigen::Vector3d e2 = v2.Eigen();
  BOOST_CHECK_EQUAL(gu::Vec3(e1 + e2), (v1 + v2));

  BOOST_CHECK_EQUAL(gu::Vec3(p1 + p2), (v1 + v2));
  BOOST_CHECK_EQUAL(gu::Vec3(p1 - p2), (v1 - v2));
  BOOST_CHECK_EQUAL(gu::Vec3(p1.cwiseProduct(p2)), (v1 % v2));
  BOOST_CHECK_EQUAL(gu::Vec3(p1.cwiseQuotient(p2)), (v1 / v2));
  BOOST_CHECK_CLOSE(p1.dot(p2), (v1 ^ v2), 1e-10);
  double s = Eigen::MatrixXd::Random(1,1)(0);
  BOOST_CHECK_EQUAL(gu::Vec3(s*p1), (s*v1));
  BOOST_CHECK_EQUAL(gu::Vec3(p1*s), (v1*s));
  BOOST_CHECK_CLOSE(p1.norm(), v1.Norm(), 1e-10);

  BOOST_CHECK_EQUAL(gu::Vec3(p1.cross(p2)), v1.Cross(v2));
  BOOST_CHECK_EQUAL(gu::Vec3(p1.cross(p2)), Cross(v1, v2));

  gu::Vec3 v3(0.1, 0.2, 0.3);
  Eigen::Vector3d p3 = v3.Eigen();
  Eigen::Vector3d p4 = gu::Eigen(v3);
  BOOST_CHECK_EQUAL(gu::Vec3(p3), gu::Vec3(p4));
  BOOST_CHECK_EQUAL(gu::Vec3(s*p3), (s*v3));

  BOOST_CHECK_EQUAL(gu::Vec3(p1/p1.norm()), v1.Normalize());

  v3 /= s;
  p3 /= s;
  BOOST_CHECK_EQUAL(v3, gu::Vec3(p3));

  v3 += v2;
  p3 += p2;
  BOOST_CHECK_EQUAL(v3, gu::Vec3(p3));

  v3 -= v2;
  p3 -= p2;
  BOOST_CHECK_EQUAL(v3, gu::Vec3(p3));

  v3 %= v2;
  p3 = p3.cwiseProduct(p2);
  BOOST_CHECK_EQUAL(v3, gu::Vec3(p3));

  v3 /= v2;
  p3 = p3.cwiseQuotient(p2);
  BOOST_CHECK_EQUAL(v3, gu::Vec3(p3));
}

BOOST_AUTO_TEST_CASE(vector4_base) {
  Eigen::Vector4d p1 = Eigen::Vector4d::Random();
  Eigen::Vector4d p2 = Eigen::Vector4d::Random();

  gu::Vec4 v1(p1);
  gu::Vec4 v2(p2);

  Eigen::Vector4d e1 = v1.Eigen();
  Eigen::Vector4d e2 = v2.Eigen();
  BOOST_CHECK_EQUAL(gu::Vec4(e1 + e2), (v1 + v2));

  BOOST_CHECK_EQUAL(gu::Vec4(p1 + p2), (v1 + v2));
  BOOST_CHECK_EQUAL(gu::Vec4(p1 - p2), (v1 - v2));
  BOOST_CHECK_EQUAL(gu::Vec4(p1.cwiseProduct(p2)), (v1 % v2));
  BOOST_CHECK_EQUAL(gu::Vec4(p1.cwiseQuotient(p2)), (v1 / v2));
  BOOST_CHECK_CLOSE(p1.dot(p2), (v1 ^ v2), 1e-10);
  double s = Eigen::MatrixXd::Random(1,1)(0);
  BOOST_CHECK_EQUAL(gu::Vec4(s*p1), (s*v1));
  BOOST_CHECK_EQUAL(gu::Vec4(p1*s), (v1*s));
  BOOST_CHECK_CLOSE(p1.norm(), v1.Norm(), 1e-10);

  gu::Vector4 v3(0.1, 0.2, 0.3, 0.4);
  Eigen::Vector4d p3 = v3.Eigen();
  Eigen::Vector4d p4 = gu::Eigen(v3);
  BOOST_CHECK_EQUAL(gu::Vec4(p3), gu::Vec4(p4));
  BOOST_CHECK_EQUAL(gu::Vec4(s*p3), (s*v3));

  BOOST_CHECK_EQUAL(gu::Vec4(p1/p1.norm()), v1.Normalize());

  v3 /= s;
  p3 /= s;
  BOOST_CHECK_EQUAL(v3, gu::Vec4(p3));

  v3 += v2;
  p3 += p2;
  BOOST_CHECK_EQUAL(v3, gu::Vec4(p3));

  v3 -= v2;
  p3 -= p2;
  BOOST_CHECK_EQUAL(v3, gu::Vec4(p3));

  v3 %= v2;
  p3 = p3.cwiseProduct(p2);
  BOOST_CHECK_EQUAL(v3, gu::Vec4(p3));

  v3 /= v2;
  p3 = p3.cwiseQuotient(p2);
  BOOST_CHECK_EQUAL(v3, gu::Vec4(p3));
}

BOOST_AUTO_TEST_CASE(quat_base) {
  Eigen::Vector4d p1 = Eigen::Vector4d::Random();
  Eigen::Vector4d p2 = Eigen::Vector4d::Random();

  gu::Quat q1 = gu::Quat(p1);
  gu::Quat q2 = gu::Quat(p2);

  BOOST_CHECK_EQUAL(p1.norm(), q1.Norm());

  p1 /= p1.norm();
  BOOST_CHECK_EQUAL(gu::Quat(p1), q1.Normalize());
}

BOOST_AUTO_TEST_CASE(matrix2x2_base) {
  Eigen::Matrix2d p1 = Eigen::Matrix2d::Random();
  Eigen::Matrix2d p2 = Eigen::Matrix2d::Random();
  Eigen::Vector2d pv = Eigen::Vector2d::Random();

  gu::Mat22 m1(p1);
  gu::Mat22 m2(p2);
  gu::Vec2 v(pv);

  Eigen::Matrix2d e1 = m1.Eigen();
  Eigen::Matrix2d e2 = m2.Eigen();
  BOOST_CHECK_EQUAL(gu::Mat22(e1 + e2), (m1 + m2));

  BOOST_CHECK_EQUAL(gu::Mat22(p1 + p2), (m1 + m2));
  BOOST_CHECK_EQUAL(gu::Mat22(p1 - p2), (m1 - m2));
  BOOST_CHECK_EQUAL(gu::Mat22(p1.cwiseProduct(p2)), (m1 % m2));
  BOOST_CHECK_EQUAL(gu::Mat22(p1.cwiseQuotient(p2)), (m1 / m2));
  BOOST_CHECK_EQUAL(gu::Mat22(p1 * p2), (m1 * m2));
  double s = Eigen::MatrixXd::Random(1,1)(0);
  BOOST_CHECK_EQUAL(gu::Mat22(s*p1), (s*m1));
  BOOST_CHECK_EQUAL(gu::Mat22(p1*s), (m1*s));
  BOOST_CHECK_CLOSE(p1.norm(), m1.Norm(), 1e-10);
  BOOST_CHECK_EQUAL(gu::Vec2(p1*pv), (m1*v));

  gu::Mat22 v3(0.1, 0.2, 0.3, 0.4);
  Eigen::Matrix2d p3 = v3.Eigen();
  Eigen::Matrix2d p4 = gu::Eigen(v3);
  BOOST_CHECK_EQUAL(gu::Mat22(p3), gu::Mat22(p4));
  BOOST_CHECK_EQUAL(gu::Mat22(s*p3), (s*v3));

  m1 += m2;
  p1 += p2;
  BOOST_CHECK_EQUAL(gu::Mat22(p1), m1);

  m1 -= m2;
  p1 -= p2;
  BOOST_CHECK_EQUAL(gu::Mat22(p1), m1);

#if 0
  gu::Vec2 sing(m1.SingularValues());
  arma::mat22 U;
  arma::vec2 si;
  arma::mat22 V;
  arma::svd(U,si,V,p1);

  BOOST_CHECK_EQUAL(gu::Vec2(si), sing);

  gu::Mat22 gmsing(1, 3, 2, 6);
  arma::mat22 amsing = gmsing.arma();

  try
  {
    gu::Mat22 tmp = gmsing.inv();
    tmp.print("Problem if printed");
  }
  catch (const std::exception& e)
  {
    std::cerr << "gu inv of singular matrix failed as expected" << std::endl;
  }

  try
  {
    arma::mat22 tmp = arma::inv(amsing);
    tmp.print("Problem if printed");
  }
  catch (const std::exception& e)
  {
    std::cerr << "arma inv of singular matrix failed as expected:" << std::endl;
  }
#endif
  puts("TODO: Test singular value decomposition!");

  BOOST_CHECK_EQUAL(gu::Mat22(p1.inverse()), gu::Inv(m1));
  BOOST_CHECK_CLOSE(p1.determinant(), m1.Det(), 1e-10);

  Eigen::Vector2d ov1 = Eigen::Vector2d::Random();
  Eigen::Vector2d ov2 = Eigen::Vector2d::Random();
  gu::Vec2 gov1(ov1), gov2(ov2);
  BOOST_CHECK_EQUAL(gu::Mat22(ov1*ov2.transpose()), gu::Outer(gov1, gov2));
}

BOOST_AUTO_TEST_CASE(matrix3x3_base) {
  Eigen::Matrix3d p1 = Eigen::Matrix3d::Random();
  Eigen::Matrix3d p2 = Eigen::Matrix3d::Random();
  Eigen::Vector3d pv = Eigen::Vector3d::Random();

  gu::Mat33 m1(p1);
  gu::Mat33 m2(p2);
  gu::Vec3 v(pv);

  BOOST_CHECK_EQUAL(m1, m1);
  BOOST_CHECK(m1 != m2);

  Eigen::Matrix3d e1 = m1.Eigen();
  Eigen::Matrix3d e2 = m2.Eigen();
  BOOST_CHECK_EQUAL(gu::Mat33(e1 + e2), (m1 + m2));

  BOOST_CHECK_EQUAL(gu::Mat33(p1 + p2), (m1 + m2));
  BOOST_CHECK_EQUAL(gu::Mat33(p1 - p2), (m1 - m2));
  BOOST_CHECK_EQUAL(gu::Mat33(p1.cwiseProduct(p2)), (m1 % m2));
  BOOST_CHECK_EQUAL(gu::Mat33(p1.cwiseQuotient(p2)), (m1 / m2));
  BOOST_CHECK_EQUAL(gu::Mat33(p1 * p2), (m1 * m2));
  double s = Eigen::MatrixXd::Random(1,1)(0);
  BOOST_CHECK_EQUAL(gu::Mat33(s*p1), (s*m1));
  BOOST_CHECK_EQUAL(gu::Mat33(p1*s), (m1*s));
  BOOST_CHECK_CLOSE(p1.norm(), m1.Norm(), 1e-10);
  BOOST_CHECK_EQUAL(gu::Vec3(p1*pv), (m1*v));

  gu::Mat33 v3(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9);

  m1 += m2;
  p1 += p2;
  BOOST_CHECK_EQUAL(gu::Mat33(p1), m1);

  m1 -= m2;
  p1 -= p2;
  BOOST_CHECK_EQUAL(gu::Mat33(p1), m1);

  BOOST_CHECK_EQUAL(gu::Mat33(p1.inverse()), gu::Inv(m1));
  BOOST_CHECK_CLOSE(p1.determinant(), m1.Det(), 1e-10);

  Eigen::Vector3d ov1 = Eigen::Vector3d::Random();
  Eigen::Vector3d ov2 = Eigen::Vector3d::Random();
  gu::Vec3 gov1(ov1), gov2(ov2);
  BOOST_CHECK_EQUAL(gu::Mat33(ov1*ov2.transpose()), gu::Outer(gov1, gov2));
}

BOOST_AUTO_TEST_CASE(matrix4x4_base) {
  Eigen::Matrix4d p1 = Eigen::Matrix4d::Random();
  Eigen::Matrix4d p2 = Eigen::Matrix4d::Random();
  Eigen::Vector4d pv = Eigen::Vector4d::Random();

  gu::Mat44 m1(p1);
  gu::Mat44 m2(p2);
  gu::Vec4 v(pv);

  BOOST_CHECK_EQUAL(m1, m1);
  BOOST_CHECK(m1 != m2);

  Eigen::Matrix4d e1 = m1.Eigen();
  Eigen::Matrix4d e2 = m2.Eigen();
  BOOST_CHECK_EQUAL(gu::Mat44(e1 + e2), (m1 + m2));

  BOOST_CHECK_EQUAL(gu::Mat44(p1 + p2), (m1 + m2));
  BOOST_CHECK_EQUAL(gu::Mat44(p1 - p2), (m1 - m2));
  BOOST_CHECK_EQUAL(gu::Mat44(p1.cwiseProduct(p2)), (m1 % m2));
  BOOST_CHECK_EQUAL(gu::Mat44(p1.cwiseQuotient(p2)), (m1 / m2));
  BOOST_CHECK_EQUAL(gu::Mat44(p1 * p2), (m1 * m2));
  double s = Eigen::MatrixXd::Random(1,1)(0);
  BOOST_CHECK_EQUAL(gu::Mat44(s*p1), (s*m1));
  BOOST_CHECK_EQUAL(gu::Mat44(p1*s), (m1*s));
  BOOST_CHECK_CLOSE(p1.norm(), m1.Norm(), 1e-10);
  BOOST_CHECK_EQUAL(gu::Vec4(p1*pv), (m1*v));

  gu::Mat44 v3(0.1, 0.2, 0.3, 0.4,
               0.5, 0.6, 0.7, 0.8,
               0.9, 1.0, 1.1, 1.2,
               1.3, 1.4, 1.5, 1.6);
  Eigen::Matrix4d p3 = v3.Eigen();
  Eigen::Matrix4d p4 = gu::Eigen(v3);
  BOOST_CHECK_EQUAL(gu::Mat44(p3), gu::Mat44(p4));
  BOOST_CHECK_EQUAL(gu::Mat44(s*p3), (s*v3));

  m1 += m2;
  p1 += p2;
  BOOST_CHECK_EQUAL(gu::Mat44(p1), m1);

  m1 -= m2;
  p1 -= p2;
  BOOST_CHECK_EQUAL(gu::Mat44(p1), m1);

  BOOST_CHECK_EQUAL(gu::Mat44(p1.inverse()), gu::Inv(m1));
  BOOST_CHECK_CLOSE(p1.determinant(), m1.Det(), 1e-10);

  Eigen::Vector4d ov1 = Eigen::Vector4d::Random();
  Eigen::Vector4d ov2 = Eigen::Vector4d::Random();
  gu::Vec4 gov1(ov1), gov2(ov2);
  BOOST_CHECK_EQUAL(gu::Mat44(ov1*ov2.transpose()), gu::Outer(gov1, gov2));
}

BOOST_AUTO_TEST_CASE(rot2_base) {
  double th1 = Eigen::MatrixXd::Random(1,1)(0);
  double th2 = Eigen::MatrixXd::Random(1,1)(0);

  Eigen::Matrix2d p1;
  p1(0, 0) = cos(th1); p1(0, 1) = -sin(th1);
  p1(1, 0) = sin(th1); p1(1, 1) = cos(th1);

  Eigen::Matrix2d p2;
  p2(0, 0) = cos(th2); p2(0, 1) = -sin(th2);
  p2(1, 0) = sin(th2); p2(1, 1) = cos(th2);

  gu::Rot2 m1(p1);
  gu::Rot2 m2(p2);

  BOOST_CHECK_EQUAL(m1, gu::Rot2(th1));
  BOOST_CHECK_EQUAL(m2, gu::Rot2(th2));

  BOOST_CHECK_EQUAL(gu::Rot2(p1*p2), (m1*m2));
  BOOST_CHECK_EQUAL(gu::Rot2(p2*p1), (m2*m1));

  gu::Rot2 r1(th1);
  gu::Rot2 r2(th2);

  BOOST_CHECK_EQUAL(r1.Error(r2), sin(th1 - th2));

  Eigen::Rotation2D<double> e1(m1.Eigen());
  Eigen::Rotation2D<double> e2(m2.Eigen());
  BOOST_CHECK_EQUAL(gu::Rot2(e1*e2), (m1*m2));
  BOOST_CHECK_EQUAL(gu::Rot2(e2*e1), (m2*m1));
  BOOST_CHECK_EQUAL(gu::Rot2(Eigen::Rotation2D<double>(th1)), gu::Rot2(th1));
}

BOOST_AUTO_TEST_CASE(rot3_base) {
  Eigen::Vector3d p = Eigen::Vector3d::Random();
  gu::Rot3 m1 = gu::Rot3(gu::Vec3(p));
  gu::Rot3 m2 = gu::ZYXToR(gu::Vec3(p));
  gu::Rot3 m3(p(0), p(1), p(2));

  BOOST_CHECK_EQUAL(m1, m2);
  BOOST_CHECK_EQUAL(m1, m3);

  BOOST_CHECK_EQUAL(m1.GetEulerZYX(), gu::Vec3(p));

  Eigen::Vector4d v1 = Eigen::Vector4d::Random();
  gu::Quat q(gu::Quat(v1).Normalize());
  m1 = gu::QuatToR(q);
  m2 = gu::Rot3(q);

  BOOST_CHECK_EQUAL(m1, m2);

  Eigen::Matrix3d p1 = gu::Eigen(m1);
  Eigen::Matrix3d p2 = gu::Eigen(m2);

  BOOST_CHECK_EQUAL(gu::Rot3(p1*p2), (m1*m2));
  BOOST_CHECK_EQUAL(gu::Rot3(p2*p1), (m2*m1));

  BOOST_CHECK_EQUAL(m1.GetEulerZYX()(0), m1.Roll());
  BOOST_CHECK_EQUAL(m1.GetEulerZYX()(1), m1.Pitch());
  BOOST_CHECK_EQUAL(m1.GetEulerZYX()(2), m1.Yaw());

  BOOST_CHECK_EQUAL(m1.GetEulerZYX(), gu::RToZYX(m1));

  Eigen::AngleAxis<double> e1(m1.Eigen());
  Eigen::AngleAxis<double> e2(m2.Eigen());
  BOOST_CHECK_EQUAL(gu::Rot3(e1*e2), (m1*m2));
  BOOST_CHECK_EQUAL(gu::Rot3(e2*e1), (m2*m1));
}

BOOST_AUTO_TEST_CASE(transform_base) {
  gu::Transform3 t1;
  gu::Transform3 t2(gu::Vec3(0, 0, 0), gu::Rot3(0, 0, 0));
  gu::Transform3 t3 = gu::Transform3::Identity();

  BOOST_CHECK_EQUAL(t1, t2);
  BOOST_CHECK_EQUAL(t1, t3);

  gu::Transform3 t(gu::Vec3(Eigen::Vector3d::Random()),
                   gu::ZYXToR(gu::Vec3(Eigen::Vector3d::Random())));

  BOOST_CHECK_EQUAL(gu::PoseUpdate(t, t1), t + t1);
}

BOOST_AUTO_TEST_CASE(matrixnxm_base) {
  typedef Eigen::Matrix<double, 4, 2> am42;
  typedef Eigen::Matrix<double, 2, 4> am24;
  typedef Eigen::Matrix<double, 4, 4> am44;
  typedef Eigen::Matrix<double, 2, 1> av2;
  typedef Eigen::Matrix<double, 4, 1> av4;

  typedef gu::MatrixNxMBase<double, 4, 2> gm42;
  typedef gu::MatrixNxMBase<double, 2, 4> gm24;
  typedef gu::MatrixNxMBase<double, 4, 4> gm44;
  typedef gu::VectorNBase<double, 2> gv2;
  typedef gu::VectorNBase<double, 4> gv4;

  am42 p1(am42::Random());
  am24 p2(am24::Random());
  am44 p3(am44::Random());
  am24 p4(am24::Random());
  av2 pv1(av2::Random());
  av4 pv2(av4::Random());

  gm42 m1(p1);
  gm24 m2(p2);
  gm44 m3(p3);
  gm24 m4(p4);
  gv2 v1(pv1);
  gv4 v2(pv2);

  BOOST_CHECK_EQUAL(gu::Vec4(p1*pv1), (m1*v1));
  BOOST_CHECK_EQUAL(gu::Vec2(p2*pv2), (m2*v2));
  BOOST_CHECK_EQUAL(gu::Mat44(p1*p2), (m1*m2));
  BOOST_CHECK_EQUAL(gu::Mat22(p2*p1), (m2*m1));

  BOOST_CHECK_EQUAL(m1, m1);
  BOOST_CHECK_EQUAL(m2, m2);
  BOOST_CHECK_EQUAL(m3, m3);
  BOOST_CHECK(m4 != m2);

  BOOST_CHECK_EQUAL(gm42(p1 + p1), (m1 + m1));
  BOOST_CHECK_EQUAL(gm24(p4 - p2), (m4 - m2));
  BOOST_CHECK_EQUAL(gm42(p1.cwiseProduct(p1)), (m1 % m1));
  BOOST_CHECK_EQUAL(gm24(p4.cwiseQuotient(p2)), (m4 / m2));
  BOOST_CHECK_EQUAL(gm44(p1 * p2), (m1 * m2));
  double s = Eigen::MatrixXd::Random(1,1)(0);
  BOOST_CHECK_EQUAL(gm24(s*p2), (s*m2));
  BOOST_CHECK_EQUAL(gm42(p1*s), (m1*s));
  BOOST_CHECK_CLOSE(p3.norm(), m3.Norm(), 1e-10);
  BOOST_CHECK_CLOSE((p1*p2).norm(), (m1*m2).Norm(), 1e-10);

  gm24 m5;
  m5(0, 0) = 0.1; m5(0, 1) = 0.2; m5(0, 2) = 0.2; m5(0, 3) = 0.3;
  m5(1, 0) = 1.1; m5(1, 1) = 1.2; m5(1, 2) = 1.2; m5(1, 3) = 1.3;

  am24 p5 = m5.Eigen();
  am24 p6 = gu::Eigen(m5);
  BOOST_CHECK_EQUAL(gm24(p5), gm24(p6));
  BOOST_CHECK_EQUAL(gm24(s*p5), (s*m5));

  m1 += m1;
  p1 += p1;
  BOOST_CHECK_EQUAL(gm42(p1), m1);

  m2 -= m4;
  p2 -= p4;
  BOOST_CHECK_EQUAL(gm24(p2), m2);

  av4 ov1 = av4::Random();
  av2 ov2 = av2::Random();
  gv4 gov1(ov1);
  gv2 gov2(ov2);
  BOOST_CHECK_EQUAL(gm42(ov1*ov2.transpose()), gu::Outer(gov1, gov2));
}
