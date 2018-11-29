#include "o2018/autonomous/test_auto.h"

namespace o2018 {
namespace autonomous {

void TestAuto::LeftSwitch() {
  SetFieldPosition(0.0, -0.3, 0.0);
  FreeArm();
  LOG(INFO, "Running TEST auto");

  SetArm(0.0, IntakeMode::INTAKE_NONE);
  StartDrivePath(2.55, 1.2, 0, 1, false);
  SetArm(45.0 * (M_PI / 180.), IntakeMode::INTAKE_NONE);
  WaitUntilDrivetrainNear(2.55, 1.3, 0.3);
  SetArm(45.0 * (M_PI / 180.), IntakeMode::OUTTAKE_FAST);
  Wait(100);
  SetArm(45.0 * (M_PI / 180.), IntakeMode::INTAKE_NONE);
}

void TestAuto::RightSwitch() {
  SetFieldPosition(0.0, -0.3, 0.0);
  FreeArm();
  LOG(INFO, "Running TEST auto");

  SetArm(0.0, IntakeMode::INTAKE_NONE);
  StartDrivePath(2.55, -1.5, 0, 1, false);
  SetArm(45.0 * (M_PI / 180.), IntakeMode::INTAKE_NONE);
  Wait(5 * 100);
  SetArm(45.0 * (M_PI / 180.), IntakeMode::OUTTAKE_FAST);
  Wait(100);
  SetArm(45.0 * (M_PI / 180.), IntakeMode::INTAKE_NONE);
}

}  // namespace autonomous
}  // namespace o2018
