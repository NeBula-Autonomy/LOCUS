#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE test_equals
#include <boost/test/unit_test.hpp>
#include <geometry_utils/GeometryUtils.h>

namespace gu = geometry_utils;

BOOST_AUTO_TEST_CASE(transform_equals_exact) {
  gu::Transform3 t1;
  Eigen::Vector3d p1 = Eigen::Vector3d::Random();
  Eigen::Vector3d p2 = Eigen::Vector3d::Random();
  t1.translation = gu::Vec3(p1);
  t1.rotation = gu::ZYXToR(gu::Vec3(p2));

  // Check that two random transforms are
  // equal after copy assignment
  gu::Transform3 t2 = t1;

  BOOST_CHECK( t1.Equals(t2) );
  BOOST_CHECK( t2.Equals(t1) );

  // Check for equivalence after
  // copy construction
  gu::Transform3 t3(t2);
  BOOST_CHECK( t3.Equals(t2) );
  BOOST_CHECK( t2.Equals(t3) );
  BOOST_CHECK( t1.Equals(t3) );
  BOOST_CHECK( t3.Equals(t1) );
}

BOOST_AUTO_TEST_CASE(transform_equals_pos_tol) {
  gu::Transform3 t1;
  Eigen::Vector3d p1 = Eigen::Vector3d::Random();
  Eigen::Vector3d p2 = Eigen::Vector3d::Random();
  t1.translation = gu::Vec3(p1);
  t1.rotation = gu::ZYXToR(gu::Vec3(p2));

  gu::Transform3 t2 = t1;

  double ptol = 1e-5;
  double ptol_element = sqrt(pow(ptol, 2.0) / 3.0);
  t2.translation(0) += ptol_element;
  t2.translation(1) += ptol_element;
  t2.translation(2) += ptol_element;

  // Check that they are not equal when slightly over the tolerance
  BOOST_CHECK( ! t1.Equals(t2, ptol - 1e-10) );
  BOOST_CHECK( ! t2.Equals(t1, ptol - 1e-10) );

  // Check that they are equal when slightly under the tolerance
  BOOST_CHECK( t1.Equals(t2, ptol + 1e-10) );
  BOOST_CHECK( t2.Equals(t1, ptol + 1e-10) );
}

BOOST_AUTO_TEST_CASE(transform_equals_rot_tol) {
  gu::Transform3 t1;
  Eigen::Vector3d p1 = Eigen::Vector3d::Random();
  Eigen::Vector3d p2 = Eigen::Vector3d::Random();
  t1.translation = gu::Vec3(p1);
  t1.rotation = gu::ZYXToR(gu::Vec3(p2));

  gu::Transform3 t2 = t1;

  double ptol = 1e-5;
  double rtol = 1e-5;

  srand(10);
  Eigen::Vector3d p3 = Eigen::Vector3d::Random();
  gu::Vector3 small_noise(1e-6*p3);

  t2.rotation = gu::ZYXToR(gu::RToZYX(t1.rotation) + small_noise);

  // Check that they equal when under tolerance
  BOOST_CHECK( t1.Equals(t2, ptol, rtol) );
  BOOST_CHECK( t2.Equals(t1, ptol, rtol) );

  gu::Vector3 big_noise = small_noise * 1e4;
  t2.rotation = gu::ZYXToR(gu::RToZYX(t1.rotation) + big_noise);

  // Check that they are unequal when slightly over tolerance
  BOOST_CHECK( ! t1.Equals(t2, ptol, rtol) );
  BOOST_CHECK( ! t2.Equals(t1, ptol, rtol) );

}

BOOST_AUTO_TEST_CASE(transform_operator_bool_eq) {
  gu::Transform3 t1;
  Eigen::Vector3d p1 = Eigen::Vector3d::Random();
  Eigen::Vector3d p2 = Eigen::Vector3d::Random();
  t1.translation = gu::Vec3(p1);
  t1.rotation = gu::ZYXToR(gu::Vec3(p2));

  // Check that two random transforms are
  // equal after copy assignment
  gu::Transform3 t2 = t1;
  BOOST_CHECK( t1 == t2 );
  BOOST_CHECK( t2 == t1 );

  // Check for equivalence after
  // copy construction
  gu::Transform3 t3(t2);
  BOOST_CHECK( t3 == t2 );
  BOOST_CHECK( t2 == t3 );
  BOOST_CHECK( t1 == t3 );
  BOOST_CHECK( t3 == t1 );
}

BOOST_AUTO_TEST_CASE(transform_operator_bool_ineq) {
  gu::Transform3 t1;
  Eigen::Vector3d p1 = Eigen::Vector3d::Random();
  t1.translation = gu::Vec3(p1);

  gu::Vector3 euler_noise = t1.translation;
  t1.rotation = gu::ZYXToR(euler_noise);

  // Check that two random transforms are
  // equal after copy assignment
  gu::Transform3 t2 = t1;
  BOOST_CHECK( ! (t1 != t2) );
  BOOST_CHECK( ! (t2 != t1) );

  // Check for equivalence after
  // copy construction
  gu::Transform3 t3(t2);
  BOOST_CHECK( ! (t3 != t2) );
  BOOST_CHECK( ! (t2 != t3) );
  BOOST_CHECK( ! (t1 != t3) );
  BOOST_CHECK( ! (t3 != t1) );

  // Make a new transform with same pos,
  // but different rot from t1, t2, t3
  gu::Transform3 t4;
  Eigen::Vector3d p2 = Eigen::Vector3d::Random();
  t4.translation = t1.translation;
  euler_noise = gu::Vec3(p2);
  t4.rotation = gu::ZYXToR(euler_noise);

  // Make sure the rotation is not
  // randomly generated close t1's rotation
  gu::Vector3 rot_err = gu::Vee(Trans(t4.rotation) * t1.rotation -
                                Trans(t1.rotation) * t4.rotation);
  double rtol = 1e-5;
  if (Norm(0.5 * rot_err) < rtol) {
    euler_noise(0) += M_PI / 4.0;
    t4.rotation = gu::ZYXToR(euler_noise);
  }

  BOOST_CHECK( t4 != t1 );
  BOOST_CHECK( t1 != t4 );

  // Make a new transform with same rot,
  // but different pos from t1, t2, t3
  gu::Transform3 t5;
  Eigen::Vector3d p3 = Eigen::Vector3d::Random();
  t5.translation = gu::Vec3(p3);
  t5.rotation = t1.rotation;

  // Make sure the translation is not
  // randomly generated close to t1's translation
  double pos_err = Norm(t5.translation - t1.translation);
  double ptol = 1e-5;
  if (pos_err < ptol)
    t5.translation(0) += ptol*10;

  BOOST_CHECK( t5 != t1 );
  BOOST_CHECK( t1 != t5 );
}
