#include "c2019/autonomous/drive_straight.h"

namespace c2019 {
namespace autonomous {

void DriveStraight::Drive() {
  SetFieldPosition(0, 0, 0.0);
  LOG(INFO, "Running NONE auto");

  StartDrivePath(3.0, 0, 0, 1, false);
  WaitUntilDriveComplete();  // :)
  ExitAutonomous();
}

}  // namespace autonomous
}  // namespace c2019
