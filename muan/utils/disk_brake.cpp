#include "muan/utils/disk_brake.h"
#include "muan/utils/math_utils.h"

namespace muan {

DiskBrake::DiskBrake(bool locked) {
  // 1/5th of a second is a good default, rounding up won't break anything.
  change_state_time_ = aos::time::Time::InMS(200);
  if (locked) {
    position_ = 1;
  } else {
    position_ = 0;
  }
  last_update_ = aos::time::Time::Now();
}

DiskBrake::DiskBrake(bool locked, units::Time change_state_time) {
  change_state_time_ = aos::time::Time::InMS(units::convert(change_state_time, units::ms));
  if (locked) {
    position_ = 1;
  } else {
    position_ = 0;
  }
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

  position_ = utils::Cap(position_, 0, 1);

  last_update_ = this_update;

  if (position_ == 1) {
    return LOCKED;
  } else if (position_ == 0) {
    return UNLOCKED;
  } else {
    return CHANGING;
  }
}

}  // namespace muan
