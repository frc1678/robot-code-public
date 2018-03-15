#include "c2018/autonomous/switch_only.h"

namespace c2018 {
namespace autonomous {

using frc971::control_loops::drivetrain::Gear;

void SwitchOnly::LeftSwitch() {
  // L - Switch is left
  LOG(INFO, "Running LEFT SWITCH ONLY auto");
  // auto_running = true;
}

void SwitchOnly::RightSwitch() {
  // R - Switch is right
  LOG(INFO, "Running RIGHT SWITCH ONLY auto");
  StartDrivePath(2.4, -1.2, 0, 1, Gear::kLowGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
  StartDrivePath(0.9, -0.2, -0.5, -1, Gear::kLowGear);
  WaitUntilDriveComplete();

  IntakeGround();

  StartDrivePath(1.6, 0, 0.5, 1, Gear::kLowGear);
  WaitForCube();

  StopIntakeGround();

  StartDrivePath(1.3, -0.5, 0.0, -1, Gear::kLowGear);
  WaitUntilDriveComplete();

  StartDrivePath(2.4, -1.2, 0, 1, Gear::kLowGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
}

}  // namespace autonomous
}  // namespace c2018
