#include "c2018/autonomous/switch_only.h"

namespace c2018 {
namespace autonomous {

using frc971::control_loops::drivetrain::Gear;

void SwitchOnly::LeftSwitch() {
  SetFieldPosition(0.0, -0.3, 0.0);
  // R - Switch is right
  LOG(INFO, "Running LEFT SWITCH ONLY auto");
  StartDrivePath(2.55, 1.2, 0, 1, Gear::kLowGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
  StartDrivePath(0.9, -0.15, -0.2, -1, Gear::kLowGear);
  WaitUntilDriveComplete();

  IntakeGround();

  StartDrivePath(1.6, -0.15, 0.0, 1, Gear::kLowGear);
  WaitForCube();

  StopIntakeGround();

  StartDrivePath(1.3, -0.15, 0.0, -1, Gear::kLowGear);
  WaitUntilDriveComplete();

  StartDrivePath(2.55, 1.2, 0, 1, Gear::kLowGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
}

void SwitchOnly::RightSwitch() {
  SetFieldPosition(0.0, -0.3, 0.0);
  // R - Switch is right
  LOG(INFO, "Running LEFT SWITCH ONLY auto");
  StartDrivePath(2.55, -1.5, 0, 1, Gear::kLowGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
  StartDrivePath(0.9, 0.0, 0.2, -1, Gear::kLowGear);
  WaitUntilDriveComplete();

  IntakeGround();

  StartDrivePath(1.6, 0.0, 0.0, 1, Gear::kLowGear);
  WaitForCube();

  StopIntakeGround();

  StartDrivePath(1.3, 0.0, 0.0, -1, Gear::kLowGear);
  WaitUntilDriveComplete();

  StartDrivePath(2.55, -1.5, 0, 1, Gear::kLowGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
}

}  // namespace autonomous
}  // namespace c2018
