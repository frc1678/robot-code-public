/*
 * MotionProfile.h
 *
 *  Created on: Sep 25, 2015
 *      Author: Kyle
 */

#ifndef SRC_ROBOTCODE_MOTIONPROFILE_H_
#define SRC_ROBOTCODE_MOTIONPROFILE_H_
#include "unitscpp/unitscpp.h"
#include <type_traits>

namespace muan {

template <typename DistanceU>
class MotionProfile {
 public:
  using VelocityU = typename std::remove_cv_t<decltype(DistanceU(0) / s)>;
  using AccelerationU =
      typename std::remove_cv_t<decltype(DistanceU(0) / s / s)>;
  MotionProfile() {}
  virtual ~MotionProfile() {}
  virtual AccelerationU CalculateSecondDerivative(Time time) = 0;
  virtual VelocityU CalculateDerivative(Time time) = 0;
  virtual DistanceU Calculate(Time time) = 0;
  virtual bool finished(Time time) = 0;
  virtual DistanceU GetTotalDistance() = 0;
};
}

#endif /* SRC_ROBOTCODE_MOTIONPROFILE_H_ */
