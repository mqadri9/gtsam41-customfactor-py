// ics variables

// class gtsam::Point2;
//class gtsam::Pose2;
//class gtsam::Vector3;

//class gtsam::Point3;
//class gtsam::Pose3;
//class gtsam::Vector6;

//class gtsam::Values;
//virtual class gtsam::noiseModel::Base;
//virtual class gtsam::NonlinearFactor;
//virtual class gtsam::NonlinearFactorGraph;
//virtual class gtsam::NoiseModelFactor : gtsam::NonlinearFactor;

#ifndef ICS_H
#define ICS_H

#include<gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/linear/NoiseModel.h>

namespace ics {

class QuadraticUnaryFactor1D : public gtsam::NoiseModelFactor1<gtsam::Vector> {
 private:
  gtsam::Vector meas_;

 public:

   void example() {
      std::cout << "an example " << std::endl;
   }
   
   QuadraticUnaryFactor1D(gtsam::Key varKey, const gtsam::Vector1 &meas, const int constraintFactorType, const gtsam::SharedNoiseModel &model = nullptr)
       : NoiseModelFactor1(model, varKey), meas_(meas)
   {
     // 0: none, 1: equality, 2: inequality
     //constraintFactorType_ = constraintFactorType;
     //isConstraintFactor_ = constraintFactorType > 0;
   }

  void evaluateErrorCustom() {
    std::cout << "return here " << std::endl;
  }

  gtsam::Vector evaluateError(const gtsam::Vector& x, boost::optional<gtsam::Matrix&> H = boost::none) const override {
    
    // compute jacobian
    gtsam::Matrix jac(1,1);
    jac << 1.;
    if (H) *H = jac;

    // compute error
    gtsam::Vector errorVector(1);
    errorVector << (x - meas_);

    return errorVector;
  }

  gtsam::Vector constraintError(const gtsam::Value& val) const override {
    gtsam::Vector x = val.cast<gtsam::Vector>();
    gtsam::Vector errorVector(1);
    errorVector <<  (x - meas_);

    return errorVector;
  }

  gtsam::Matrix constraintJacobian(const gtsam::Value& val) const override  {
    
    gtsam::Vector x = val.cast<gtsam::Vector>();

    gtsam::Matrix jac(1,1);
    jac << 1.;
    gtsam::Matrix G = jac;

    return G;
  }

};

class QuadraticBinaryFactor1D : public gtsam::NoiseModelFactor2<gtsam::Vector, gtsam::Vector> {
 private:
  gtsam::Vector meas_;

 public:
   QuadraticBinaryFactor1D(gtsam::Key varKey1, gtsam::Key varKey2, const gtsam::Vector1 &meas, const gtsam::SharedNoiseModel &model = nullptr)
       : NoiseModelFactor2(model, varKey1, varKey2), meas_(meas)
   {}

   gtsam::Vector evaluateError(const gtsam::Vector &x1, const gtsam::Vector &x2, boost::optional<gtsam::Matrix &> H1 = boost::none, boost::optional<gtsam::Matrix &> H2 = boost::none) const override
   {
       // compute jacobian
       if (H1) *H1 = (gtsam::Matrix11() << -1.).finished();
       if (H2) *H2 = (gtsam::Matrix11() << 1.).finished();

    // compute error
    gtsam::Vector errorVector(1);
    errorVector << ((x2 - x1) - meas_);
    
    return errorVector;
   }
};

}  // namespace ics

#endif 

