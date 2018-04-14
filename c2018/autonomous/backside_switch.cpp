#include "c2018/autonomous/backside_switch.h"

namespace c2018 {
namespace autonomous {

using muan::units::deg;
using frc971::control_loops::drivetrain::Gear::kLowGear;
using frc971::control_loops::drivetrain::Gear::kHighGear;

constexpr double kStartY = 3.0;

void BacksideSwitch::SwitchBack() {
  SetFieldPosition(0.0, kStartY, 180 * deg);
  // To backside of switch
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  StartDrivePath(kCubeX + 1.0, 2.6, 180 * deg, -1, kHighGear);
  WaitUntilDriveComplete();
  // Into switch
  StartDrivePath(kCubeX, 2.1, 200 * deg, 1, kHighGear);
  WaitUntilDriveComplete();

  Score(true);
  Wait(50);
  // Back off
  StartDrivePath(kCubeX + 1.2, 2.1, 180 * deg, -1, kHighGear, 0.2);
  Wait(50);
  IntakeGround();
  IntakeOpen();
  WaitUntilDriveComplete();

  StartDrivePath(kCubeX, 2.1, 180 * deg, 1, kHighGear);
  WaitUntilDrivetrainNear(kCubeX, 2.1, 0.1);
  IntakeClose();
  WaitForCube();
  WaitUntilDriveComplete();

  StartDriveRelative(-0.3, 0.0);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  Wait(100);
  StartDriveRelative(0.3, 0.0);
  WaitUntilDriveComplete();
  Score(true);

  StartDriveRelative(-1.0, 0.0);
  Wait(50);
  IntakeGround();
  WaitUntilDriveComplete();

  StartDrivePath(kCubeX + 0.2, 1.25, 200 * deg, 1, kHighGear, 0.0, 0.3);
  WaitForCube();
  WaitUntilDriveComplete();

  StartDrivePath(kCubeX, 2.1, 200 * deg, -1, kHighGear, 0.0, 0.4);
}

}  // namespace autonomous
}  // namespace c2018
