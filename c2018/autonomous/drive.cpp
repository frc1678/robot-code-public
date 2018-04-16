#include "c2018/autonomous/drive.h"

namespace c2018 {
namespace autonomous {

using frc971::control_loops::drivetrain::Gear;

void Drive::DriveBackwards() {
  SetFieldPosition(0.0, 0.0, 0.0);
  LOG(INFO, "Running DRIVE BACKWARDS auto");

  StartDriveAtAngle(-4.0, 0);  // Drive backwards
  WaitUntilDriveComplete();
}

}  // namespace autonomous
}  // namespace c2018
