#include "c2018/autonomous/drive.h"

namespace c2018 {
namespace autonomous {

using frc971::control_loops::drivetrain::Gear;

void Drive::Drive() {
  SetFieldPosition(0.0, 0.0, 0.0);

  StartDriveAtAngle(3.0, 0 * deg);  // Drive forwards
  WaitUntilDriveComplete();
}

}  // namespace autonomous
}  // namespace c2018
