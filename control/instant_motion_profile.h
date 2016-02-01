/*
 * InstantMotionProfile.h
 * Just add water!
 *
 *  Created on: Sep 25, 2015
 *      Author: Wesley
 */

#ifndef MUAN_CONTROL_INSTANTMOTIONPROFILE_H_
#define MUAN_CONTROL_INSTANTMOTIONPROFILE_H_

#include "motion_profile.h"
#include <cmath>

namespace muan {

template <typename DistanceU>
class InstantMotionProfile : public MotionProfile<DistanceU> {
  typedef Units<DistanceU::u1, DistanceU::u2 - 1, DistanceU::u3, DistanceU::u4>
      VelocityU;
  typedef Units<DistanceU::u1, DistanceU::u2 - 2, DistanceU::u3, DistanceU::u4>
      AccelerationU;

  DistanceU distance_;
  Time time_;

 public:
  InstantMotionProfile<DistanceU>(DistanceU distance, Time time) {
    distance_ = distance;
    time_ = time;
  }
  virtual ~InstantMotionProfile() {}
  VelocityU CalculateDerivative(Time time) override { return VelocityU(0); }
  DistanceU Calculate(Time time) override { return distance_; }
  bool finished(Time time) override { return time > time_; }
};
}

#endif /* SRC_ROBOTCODE_TRAPEZOIDALMOTIONPROFILE_H_ */
