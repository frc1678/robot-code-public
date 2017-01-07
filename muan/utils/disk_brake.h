#ifndef MUAN_UTILS_DISK_BRAKE_H_
#define MUAN_UTILS_DISK_BRAKE_H_

#include "muan/units/units.h"
#include "third_party/aos/common/time.h"

namespace muan {

class DiskBrake {
 public:
  DiskBrake(bool locked);
  DiskBrake(bool locked, units::Time change_state_time);
  enum BrakeState { LOCKED, UNLOCKED, CHANGING };
  // Update state based on whether it is being told to lock and returns a BrakeState
  BrakeState Update(bool locking);

 protected:
  // 1 is locked, 0 is unlocked, in between is changing
  double position_;
  // Time from locked to unlocked, or unlocked to locked.
  aos::time::Time change_state_time_;
  // When Update() was last called
  aos::time::Time last_update_;
};

}  // namespace muan

#endif  // MUAN_UTILS_DISK_BRAKE_H_
