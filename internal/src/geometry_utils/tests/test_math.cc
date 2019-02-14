#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_math
#include <boost/test/unit_test.hpp>

#include <ros/ros.h>
#include <geometry_utils/GeometryUtilsMath.h>

namespace gu = geometry_utils;
namespace gm = gu::math;

BOOST_AUTO_TEST_CASE(math) {
  srand(time(NULL));
  float rf = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
  double rd = static_cast<double>(rand())/static_cast<double>(RAND_MAX);

  BOOST_CHECK_EQUAL(gm::cos<float>(rf), cosf(rf));
  BOOST_CHECK_EQUAL(gm::cos<float>(rf), gm::cos(rf));
  BOOST_CHECK_EQUAL(gm::cos<double>(rd), cos(rd));
  BOOST_CHECK_EQUAL(gm::cos<double>(rd), gm::cos(rd));

  BOOST_CHECK_EQUAL(gm::acos<float>(rf), acosf(rf));
  BOOST_CHECK_EQUAL(gm::acos<float>(rf), gm::acos(rf));
  BOOST_CHECK_EQUAL(gm::acos<double>(rd), acos(rd));
  BOOST_CHECK_EQUAL(gm::acos<double>(rd), gm::acos(rd));

  BOOST_CHECK_EQUAL(gm::sin<float>(rf), sinf(rf));
  BOOST_CHECK_EQUAL(gm::sin<float>(rf), gm::sin(rf));
  BOOST_CHECK_EQUAL(gm::sin<double>(rd), sin(rd));
  BOOST_CHECK_EQUAL(gm::sin<double>(rd), gm::sin(rd));

  BOOST_CHECK_EQUAL(gm::asin<float>(rf), asinf(rf));
  BOOST_CHECK_EQUAL(gm::asin<float>(rf), gm::asin(rf));
  BOOST_CHECK_EQUAL(gm::asin<double>(rd), asin(rd));
  BOOST_CHECK_EQUAL(gm::asin<double>(rd), gm::asin(rd));

  BOOST_CHECK_EQUAL(gm::tan<float>(rf), tanf(rf));
  BOOST_CHECK_EQUAL(gm::tan<float>(rf), gm::tan(rf));
  BOOST_CHECK_EQUAL(gm::tan<double>(rd), tan(rd));
  BOOST_CHECK_EQUAL(gm::tan<double>(rd), gm::tan(rd));

  BOOST_CHECK_EQUAL(gm::fabs<float>(rf), fabsf(rf));
  BOOST_CHECK_EQUAL(gm::fabs<float>(rf), gm::fabs(rf));
  BOOST_CHECK_EQUAL(gm::fabs<double>(rd), fabs(rd));
  BOOST_CHECK_EQUAL(gm::fabs<double>(rd), gm::fabs(rd));

  BOOST_CHECK_EQUAL(gm::sqrt<float>(rf), sqrtf(rf));
  BOOST_CHECK_EQUAL(gm::sqrt<float>(rf), gm::sqrt(rf));
  BOOST_CHECK_EQUAL(gm::sqrt<double>(rd), sqrt(rd));
  BOOST_CHECK_EQUAL(gm::sqrt<double>(rd), gm::sqrt(rd));

  float rf1 = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
  float rf2 = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
  double rd1 = static_cast<double>(rand())/static_cast<double>(RAND_MAX);
  double rd2 = static_cast<double>(rand())/static_cast<double>(RAND_MAX);

  BOOST_CHECK_EQUAL(gm::atan2<float>(rf1, rf2), atan2f(rf1, rf2));
  BOOST_CHECK_EQUAL(gm::atan2<float>(rf1, rf2), gm::atan2(rf1, rf2));
  BOOST_CHECK_EQUAL(gm::atan2<double>(rd1, rd2), atan2(rd1, rd2));
  BOOST_CHECK_EQUAL(gm::atan2<double>(rd1, rd2), gm::atan2(rd1, rd2));

  BOOST_CHECK_EQUAL(gm::hypot<float>(rf1, rf2), hypotf(rf1, rf2));
  BOOST_CHECK_EQUAL(gm::hypot<float>(rf1, rf2), gm::hypot(rf1, rf2));
  BOOST_CHECK_EQUAL(gm::hypot<double>(rd1, rd2), hypot(rd1, rd2));
  BOOST_CHECK_EQUAL(gm::hypot<double>(rd1, rd2), gm::hypot(rd1, rd2));
}
