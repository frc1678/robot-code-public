#include "c2019/autonomous/test_auto.h"

namespace c2019 {
namespace autonomous {

void TestAuto::Run() {
  SetFieldPosition(0.0, -0.3, 0.0);
  LOG(INFO, "Running TEST auto");

  StartDrivePath(2.55, 1.2, 0, 1, false);
  WaitUntilDriveComplete();
  ExitAutonomous();
}

}  // namespace autonomous
}  // namespace c2019
