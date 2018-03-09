#include "muan/control/ramping.h"
#include <cmath>

namespace muan {

namespace control {

Ramping::Ramping(double acceleration, double initial, bool deccelerate)
    : acceleration_{acceleration},
      profiled_goal_{initial},
      unprofiled_goal_{initial},
      deccelerate_{deccelerate} {}

double Ramping::Update(double goal) {
  unprofiled_goal_ = goal;
  if (unprofiled_goal_ < profiled_goal_) {
    if (deccelerate_) {
      profiled_goal_ =
          std::fmax(profiled_goal_ - acceleration_, unprofiled_goal_);
    } else {
      profiled_goal_ = unprofiled_goal_;
    }
  } else {
    profiled_goal_ =
        std::fmin(profiled_goal_ + acceleration_, unprofiled_goal_);
  }
  return profiled_goal_;
}

}  // namespace control

}  // namespace muan
