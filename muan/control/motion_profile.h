#ifndef SRC_ROBOTCODE_MOTIONPROFILE_H_
#define SRC_ROBOTCODE_MOTIONPROFILE_H_
#include "muan/utils/math_utils.h"
#include "unitscpp/unitscpp.h"
#include <type_traits>

namespace muan {

namespace control {

template <typename DistanceType>
struct MotionProfilePosition {
  DistanceType position;
  TimeDerivative<DistanceType> velocity;
};

/*
 * A base class for a motion profile.
 * To use, implement Calculate(Time) and total_time()
 */
template <typename DistanceType>
class MotionProfile {
 public:
  MotionProfile() = default;
  virtual ~MotionProfile() = default;

  virtual MotionProfilePosition<DistanceType> Calculate(Time time) const = 0;

  virtual Time total_time() const = 0;
  virtual bool finished(Time time) { return time > total_time(); }
};

} /* control */

} /* muan */

#endif /* SRC_ROBOTCODE_MOTIONPROFILE_H_ */
