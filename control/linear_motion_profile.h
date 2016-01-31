#include "motion_profile.h"

namespace muan {

template <class DistanceU>
class LinearMotionProfile : public MotionProfile<DistanceU> {
 private:
  typedef Units<DistanceU::u1, DistanceU::u2 - 1, DistanceU::u3, DistanceU::u4>
      VelocityU;
  typedef Units<DistanceU::u1, DistanceU::u2 - 2, DistanceU::u3, DistanceU::u4>
      AccelerationU;
  DistanceU distance_;
  Time time_;

 public:
  LinearMotionProfile(DistanceU distance, Time t)
      : distance_(distance), time_(t) {}
  LinearMotionProfile(DistanceU distance, VelocityU v)
      : distance_(distance), time_(distance / v) {}
  DistanceU Calculate(Time t) {
    return t > time_ ? distance_ : t * distance_ / time_;
  }
  VelocityU CalculateDerivative(Time t) {
    return t > time_ ? VelocityU(0) : distance_ / time_;
  }
  bool finished(Time t) { return t >= time_; }
};
}
