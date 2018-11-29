#include "o2018/autonomous/drive_straight.h"

namespace o2018 {
namespace autonomous {

void DriveStraight::Drive() {
  SetFieldPosition(0, 0, 0.0);
  LOG(INFO, "Running NONE auto");
  FreeArm();

  StartDrivePath(3.0, 0, 0, 1, false);
  WaitUntilDriveComplete();  // :)
}

}  // namespace autonomous
}  // namespace o2018
