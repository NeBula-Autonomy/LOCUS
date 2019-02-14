#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_equals
#include <boost/test/unit_test.hpp>

#include <ros/ros.h>
#include <geometry_utils/GeometryUtils.h>

namespace gu = geometry_utils;

BOOST_AUTO_TEST_CASE(so3_matrix) {
  gu::Rot3 r1(0, 0, 0);
  gu::Rot3 r2(0, 0, 0);
  gu::Rot3 r3(M_PI/2.0, 0, 0);

  BOOST_CHECK_EQUAL( r1.Error(r2), 0.0 );
  BOOST_CHECK_EQUAL( r1.Error(r3), 1.0 );
}

BOOST_AUTO_TEST_CASE(so3_quat) {
  gu::Quat q1 = gu::RToQuat(gu::Rot3(0, 0, 0));
  gu::Quat q2 = gu::RToQuat(gu::Rot3(0, 0, 0));
  gu::Quat q3 = gu::RToQuat(gu::Rot3(M_PI/2.0, 0, 0));

  gu::Rot3 r1(q1);
  gu::Rot3 r2(q2);
  gu::Rot3 r3(q3);

  gu::Vec3 v1 = r1.Row(2);
  gu::Vec3 v2 = r2.Row(2);
  gu::Vec3 v3 = r3.Row(2);

  BOOST_CHECK_EQUAL( acos(v1.Dot(v2)), 0.0 );
  BOOST_CHECK_CLOSE( acos(v1.Dot(v3)), M_PI/2.0 , 1e-10);

  BOOST_CHECK_CLOSE( q1.Error(q2).AxisAngle()(0), 0.0, 1e-10);
  BOOST_CHECK_CLOSE( q1.Error(q3).AxisAngle()(0), M_PI/2, 1e-10);

  BOOST_CHECK_CLOSE( acos(v1.Dot(v2)), q1.Error(q2).AxisAngle()(0) , 1e-10);
  BOOST_CHECK_CLOSE( acos(v1.Dot(v3)), q1.Error(q3).AxisAngle()(0) , 1e-10);
}
