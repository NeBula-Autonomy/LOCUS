/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *  @file  BetweenChordalFactor.h
 *  @author Siddharth Choudhary
 **/
#pragma once

#include <ostream>

#include <gtsam/base/Testable.h>
#include <gtsam/base/Lie.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace gtsam {

  typedef Eigen::Matrix<double, 9, 1> Vector9;


  /**
   * A class for a measurement predicted by "between(config[key1],config[key2])", uses Chordal relaxation
   * @tparam VALUE the Value type
   * @addtogroup SLAM
   */
  template<class VALUE>
  class BetweenChordalFactor: public NoiseModelFactor2<VALUE, VALUE> {

    // Check that VALUE type is a testable Lie group
    GTSAM_CONCEPT_LIE_TYPE(VALUE)
    GTSAM_CONCEPT_TESTABLE_TYPE(VALUE)


  public:

    typedef VALUE T;

  private:

    typedef BetweenChordalFactor<VALUE> This;
    typedef NoiseModelFactor2<VALUE, VALUE> Base;

    VALUE measured_; /** The measurement */


  public:

    // shorthand for a smart pointer to a factor
    typedef typename boost::shared_ptr<BetweenChordalFactor> shared_ptr;


    /** default constructor - only use for serialization */
    BetweenChordalFactor() {}

    /** Constructor */
    BetweenChordalFactor(Key key1, Key key2, const VALUE& measured,
                         const SharedNoiseModel& model) :
      Base(model, key1, key2), measured_(measured) {
    }

    virtual ~BetweenChordalFactor() {}

    /// @return a deep copy of this factor
    virtual gtsam::NonlinearFactor::shared_ptr clone() const {
      return boost::static_pointer_cast<gtsam::NonlinearFactor>(
          gtsam::NonlinearFactor::shared_ptr(new This(*this))); }

    /** implement functions needed for Testable */

    /** print */
    virtual void print(const std::string& s, const KeyFormatter& keyFormatter = DefaultKeyFormatter) const {
      std::cout << s << "BetweenChordalFactor("
          << keyFormatter(this->key1()) << ","
          << keyFormatter(this->key2()) << ")\n";
      measured_.print("  measured: ");
      this->noiseModel_->print("  noise model: ");
    }

    /** equals */
    virtual bool equals(const NonlinearFactor& expected, double tol=1e-9) const {
      const This *e =  dynamic_cast<const This*> (&expected);
      return e != NULL && Base::equals(*e, tol) && this->measured_.equals(e->measured_, tol);
    }


    Vector convertToVector(Matrix R){
      return (gtsam::Vector(9) << R(0,0), R(0,1), R(0,2),/*  */ R(1,0), R(1,1), R(1,2), /*  */ R(2,0), R(2,1), R(2,2)).finished();
    }

    /** vector of errors */
  Vector evaluateError(const T& pi, const T& pj, boost::optional<Matrix&> Hi =
      boost::none, boost::optional<Matrix&> Hj = boost::none) const {

    Matrix3 S1 = skewSymmetric(-1,0,0); //(0 0 0, 0 0 1, 0 -1 0);
    Matrix3 S2 = skewSymmetric(0,-1,0); //sparse([0 0 -1; 0 0 0; 1  0 0]);
    Matrix3 S3 = skewSymmetric(0,0,-1); //sparse([0 1  0;-1 0 0; 0  0 0]);

    Matrix3 Rij = measured_.rotation().matrix();
    Matrix3 Rijt = Rij.transpose();
    Vector3 tij = measured_.translation().vector();


    Matrix3 Ri = pi.rotation().matrix();
    Vector3 ti = pi.translation().vector();

    Matrix3 Rj = pj.rotation().matrix();
    Vector3 tj = pj.translation().vector();

    Matrix3 Ri_Rij = Ri*Rij;
    Matrix3 error_R = Ri_Rij - Rj;
    Matrix3 Rt = error_R.transpose();
    Vector9 error_r = (gtsam::Vector(9) << Rt(0,0), Rt(0,1), Rt(0,2),/*  */ Rt(1,0), Rt(1,1), Rt(1,2), /*  */ Rt(2,0), Rt(2,1), Rt(2,2)).finished(); //convertToVector(error_R.transpose()); // stack by columns
    Vector3 error_t = tj - ti - Ri * tij;

    // fill in residual error vector
    Vector error = (Vector(12) << error_r, error_t).finished();

    if(Hi||Hj){

      // fill in Jacobian wrt pi
      *Hi = Matrix::Zero(12,6);
      Hi->block(0,0,3,3) = Ri_Rij*S1*Rijt;
      Hi->block(3,0,3,3) = Ri_Rij*S2*Rijt;
      Hi->block(6,0,3,3) = Ri_Rij*S3*Rijt;
      Hi->block(9,0,3,3) = Ri*skewSymmetric(tij);
      Hi->block(9,3,3,3) = -eye(3); // TODO: define once outside

      // fill in Jacobian wrt pj
      *Hj = Matrix::Zero(12,6);
      Hj->block(0,0,3,3) = - Rj*S1;
      Hj->block(3,0,3,3) = - Rj*S2;
      Hj->block(6,0,3,3) = - Rj*S3;
      Hj->block(9,3,3,3) = eye(3);  // TODO: define once outside

    }

    return error;
    }

    /** return the measured */
    const VALUE& measured() const {
      return measured_;
    }

    /** number of variables attached to this factor */
    std::size_t size() const {
      return 2;
    }

  private:

    /** Serialization function */
    friend class boost::serialization::access;
    template<class ARCHIVE>
    void serialize(ARCHIVE & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("NoiseModelFactor2",
          boost::serialization::base_object<Base>(*this));
      ar & BOOST_SERIALIZATION_NVP(measured_);
    }
  }; // \class BetweenChordalFactor

} /// namespace gtsam
