#ifndef _MUAN_UTILS_DISK_BRAKE_H
#define _MUAN_UTILS_DISK_BRAKE_H

#include "muan/units/units.h"
#include "third_party/aos/common/time.h"

namespace muan {

class DiskBrake {
 public:
  DiskBrake();
  enum BrakeState {
    LOCKED,
    UNLOCKED,
    CHANGING
  };
  // Update state based on whether it is being told to lock and returns a BrakeState
  BrakeState Update(bool locking);
 protected:
  // 1 is locked, 0 is unlocked, in between is changing
  double position_;
  aos::time::Time change_state_time_;
  aos::time::Time last_update_;
};

} // namespace muan

#endif // _MUAN_UTILS_BRAKE_H
