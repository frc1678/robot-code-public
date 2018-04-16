#include "c2018/autonomous/none.h"

namespace c2018 {
namespace autonomous {

using frc971::control_loops::drivetrain::Gear;

void None::NoneAuto() {
  SetFieldPosition(1678.1678, 1678.1678, 0.0);
  LOG(INFO, "Running NONE auto");

  Wait(1678);  // :)
}

}  // namespace autonomous
}  // namespace c2018
