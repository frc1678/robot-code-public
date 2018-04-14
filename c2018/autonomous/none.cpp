#include "c2018/autonomous/none.h"

namespace c2018 {
namespace autonomous {

using frc971::control_loops::drivetrain::Gear;

void None::None() {
  SetFieldPosition(1678.1678, 1678.1678, 0.0);

  Wait(1678);  // :)
}

}  // namespace autonomous
}  // namespace c2018
