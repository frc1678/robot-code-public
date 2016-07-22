#ifndef SRC_ROBOTCODE_MOTIONPROFILE_H_
#define SRC_ROBOTCODE_MOTIONPROFILE_H_
#include "unitscpp/unitscpp.h"
#include <type_traits>

namespace muan {

namespace control {

template <typename T>
using TimeDerivative = std::remove_cv_t<decltype(T{0} / s)>;

template <typename T>
using TimeDerivative2 = std::remove_cv_t<decltype(T{0} / s / s)>;

template <typename DistanceType>
struct MotionProfilePosition {
  DistanceType position;
  TimeDerivative<DistanceType> velocity;
};

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
