#ifndef MUAN_CONTROL_MOTION_PROFILE_H_
#define MUAN_CONTROL_MOTION_PROFILE_H_
#include <type_traits>
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"

namespace muan {

namespace control {

struct MotionProfilePosition {
  muan::units::Length position;
  muan::units::Velocity velocity;
};

/*
 * A base class for a motion profile.
 * To use, implement Calculate(Time) and total_time()
 */
class MotionProfile {
 public:
  MotionProfile() = default;
  virtual ~MotionProfile() = default;

  virtual MotionProfilePosition Calculate(muan::units::Time time) const = 0;

  virtual muan::units::Time total_time() const = 0;
  virtual bool finished(muan::units::Time time) { return time > total_time(); }
};

}  // namespace control

}  // namespace muan

#endif  // MUAN_CONTROL_MOTION_PROFILE_H_
