#include "disk_brake.h"

namespace muan {

DiskBrake::DiskBrake() {
  // 1/5th of a second is a good default, rounding up won't break anything.
  change_state_time_ = aos::time::Time::InMS(200);
  position_ = 1;
  last_update_ = aos::time::Time::Now();
}

DiskBrake::BrakeState DiskBrake::Update(bool locking) {
  aos::time::Time this_update = aos::time::Time::Now();
  aos::time::Time dt = this_update - last_update_;

  if (locking) {
    position_ += dt / change_state_time_;
  } else {
    position_ -= dt / change_state_time_;
  }

  if (position_ > 1) {
    position_ = 1;
  }
  if(position_ < 0) {
    position_ = 0;
  }

  last_update_ = this_update;

  if (position_ == 1) {
    return LOCKED;
  } else if (position_ == 0) {
    return UNLOCKED;
  } else {
    return CHANGING;
  }
}

} // namespace muan
