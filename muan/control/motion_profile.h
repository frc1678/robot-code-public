#ifndef MUAN_CONTROL_MOTION_PROFILE_H_
#define MUAN_CONTROL_MOTION_PROFILE_H_
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"
#include <type_traits>

namespace muan {

namespace control {

using namespace muan::units;

struct MotionProfilePosition {
  Length position;
  Velocity velocity;
};

/*
 * A base class for a motion profile.
 * To use, implement CalculateFromElapsed(Time) and total_time()
 */
class MotionProfile {
 public:
  MotionProfile(Time initial_time = 0.0 * s) : initial_time_{initial_time} {}
  virtual ~MotionProfile() = default;

  MotionProfilePosition Calculate(Time time) {
    return CalculateFromElapsed(time - initial_time_);
  }

  virtual Time total_time() const = 0;
  Time final_time() const { return total_time() + initial_time_; }
  virtual bool finished(Time time) const { return time > final_time(); }

 protected:
  // Calculate the required state from the time elapsed since the beginning of
  // the profile
  virtual MotionProfilePosition CalculateFromElapsed(
      Time elapsed_time) const = 0;

 private:
  Time initial_time_;
};

}  // namespace control

}  // namespace muan

#endif /* MUAN_CONTROL_MOTION_PROFILE_H_ */
