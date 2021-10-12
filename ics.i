// ics variables


#include <cpp/ics2.h>

namespace ics {
 
  class QuadraticUnaryFactor1D: gtsam::NoiseModelFactor {
    void example();
    QuadraticUnaryFactor1D(gtsam::Key varKey, const gtsam::Vector1 &meas, const int constraintFactorType, const gtsam::SharedNoiseModel &model = nullptr);
    gtsam::Vector constraintError(const gtsam::Value& val);
    gtsam::Matrix constraintJacobian(const gtsam::Value& val);
    gtsam::Vector evaluateError(const gtsam::Vector& x);
    void evaluateErrorCustom();
  };

  class QuadraticBinaryFactor1D: gtsam::NoiseModelFactor {
    QuadraticBinaryFactor1D(gtsam::Key varKey1, gtsam::Key varKey2, const gtsam::Vector1 &meas, const gtsam::SharedNoiseModel &model = nullptr);
    gtsam::Vector evaluateError(const gtsam::Vector &x1, const gtsam::Vector &x2);
  };


}
// namespace ics
