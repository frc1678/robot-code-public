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
  StartDrivePath(kCubeX + 0.6, 2.8, 200 * deg, -1, kHighGear);
  WaitUntilDriveComplete();

  // Into switch
  StartDrivePath(kCubeX, 2.1, 200 * deg, 1, kHighGear);
  WaitUntilDriveComplete();

  Score(true);
  Wait(50);

  // Back off
  StartDriveRelative(-0.3, 0.0, 1.0);
  Wait(100);
  IntakeGround();
  IntakeOpen();
  WaitUntilDriveComplete();

  // Get first platform cube
  StartDriveRelative(0.3, 0.0);
  WaitForCube();
  IntakeClose();

  // Back off
  StartDriveRelative(-0.3, 0.0, 1.0);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  //Score second cube
  WaitUntilDriveComplete();
  StartDriveRelative(0.4, 0.0);
  WaitUntilDriveComplete();
  Score(true);

  // Back off
  StartDriveRelative(-0.55, 0.0, 1.0);
  WaitUntilDriveComplete();
  IntakeGround();

  // Get second platform cube
  StartDriveAtAngle(0.7, 55 * deg, 0.0);
  WaitForCube();

  // Back off
  StartDriveAtAngle(-0.4, 0 * deg, 1.0);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  Wait(50);

  // Score third cube
  StartDriveRelative(0.45, 10 * deg);
  WaitUntilDriveComplete();
  Score(true);
}

}  // namespace autonomous
}  // namespace c2018
