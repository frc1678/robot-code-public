#include "c2018/autonomous/switch_and_scale.h"

#include <cmath>

namespace c2018 {
namespace autonomous {

using muan::units::deg;
using frc971::control_loops::drivetrain::Gear::kLowGear;
using frc971::control_loops::drivetrain::Gear::kHighGear;

void SwitchAndScale::LeftSwitchLeftScale() {
  SetFieldPosition(0.0, -0.3, 0.0);
  // LL - Left Switch Left Scale
  LOG(INFO, "Running LEFT SWITCH LEFT SCALE auto");
  // To switch
  StartDrivePath(2.55, 1.2, 0, 1, kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  // Align to pyramid
  StartDrivePath(1.2, 0.7, -45 * deg, -1, kHighGear);
  Wait(175);
  IntakeGround();
  WaitUntilDriveComplete();
  // Drive into cube
  StartDrivePath(1.9, 0.2, -45 * deg, 1, kHighGear);
  WaitForCube();
  WaitUntilDriveComplete();
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  // Back off and go away
  StartDrivePath(8.0, 3.5, -180 * deg, -1, kHighGear, 0.2);
  WaitUntilDrivetrainNear(5.0, 3.3, 1.0);
  StartDrivePath(8.3, 3.3, -150 * deg, -1, kHighGear, 0.5);
  WaitUntilDriveComplete();
  // Go up
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_HIGH_FORWARD);
  // Turn in place
  StartDriveAtAngle(0.0, -90 * deg);
  WaitUntilDriveComplete();
  Score(false);
  Wait(100);
  // Turn in place
  StartDriveRelative(-0.5, 60 * deg);
  Wait(100);
  // Go to intake
  IntakeGround();
  WaitUntilDriveComplete();
}

void SwitchAndScale::RightSwitchRightScale() {
  SetFieldPosition(0.0, -0.3, 0.0);
  // LL - Left Switch Left Scale
  LOG(INFO, "Running LEFT SWITCH LEFT SCALE auto");
  // To switch
  StartDrivePath(2.55, -1.2, 0, 1, kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  // Align to pyramid
  StartDrivePath(1.2, -0.7, 45 * deg, -1, kHighGear);
  Wait(175);
  IntakeGround();
  WaitUntilDriveComplete();
  // Drive into cube
  StartDrivePath(1.9, -0.2, 45 * deg, 1, kHighGear);
  WaitForCube();
  WaitUntilDriveComplete();
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  // Back off and go away
  StartDrivePath(8.0, -3.5, 180 * deg, -1, kHighGear, 0.2);
  WaitUntilDrivetrainNear(5.0, -3.5, 1.0);
  StartDrivePath(8.3, -3.5, 150 * deg, -1, kHighGear, 0.5);
  WaitUntilDriveComplete();
  // Go up
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_HIGH_FORWARD);
  // Turn in place
  StartDriveAtAngle(0.0, 90 * deg);
  WaitUntilDriveComplete();
  Score(false);
  Wait(100);
  // Turn in place
  StartDriveRelative(-0.5, -60 * deg);
  Wait(100);
  // Go to intake
  IntakeGround();
  WaitUntilDriveComplete();
}

void SwitchAndScale::RightSwitchLeftScale() {
  SetFieldPosition(0.0, -0.3, 0.0);
  // LL - Left Switch Left Scale
  LOG(INFO, "Running LEFT SWITCH LEFT SCALE auto");
  // To switch
  StartDrivePath(2.55, -1.2, 0, 1, kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  // Align to pyramid
  StartDrivePath(1.2, -0.7, 45 * deg, -1, kHighGear);
  Wait(175);
  IntakeGround();
  WaitUntilDriveComplete();
  // Drive into cube
  StartDrivePath(1.9, -0.2, 45 * deg, 1, kHighGear);
  WaitForCube();
  WaitUntilDriveComplete();
  StartDrivePath(1.7, -0.5, 100 * deg, -1, kHighGear);
  WaitUntilDriveComplete();
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  // Back off and go away
  StartDrivePath(8.0, 3.5, 0 * deg, 1, kHighGear);
  WaitUntilDrivetrainNear(5.0, 2.5, 1.0);
  StartDrivePath(8.3, 3.3, -30 * deg, 1, kHighGear);
  WaitUntilDriveComplete();
  // Go up
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_HIGH_FORWARD);
  // Turn in place
  StartDriveAtAngle(0.0, -90 * deg);
  WaitUntilDriveComplete();
  Score(false);
  Wait(100);
  // Turn in place
  StartDriveRelative(-0.5, 60 * deg);
  Wait(100);
  // Go to intake
  IntakeGround();
  WaitUntilDriveComplete();
}

void SwitchAndScale::LeftSwitchRightScale() {
  SetFieldPosition(0.0, -0.3, 0.0);
  // LL - Left Switch Right Scale
  LOG(INFO, "Running LEFT SWITCH RIGHT SCALE auto");
  StartDrivePath(2.55, 1.2, 0, 1, kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
  StartDrivePath(0.9, -0.15, -0.2, -1, kHighGear);
  WaitUntilDriveComplete();
  IntakeGround();
  StartDrivePath(1.6, -0.15, 0.0, 1, kHighGear);
  WaitForCube();
  StopIntakeGround();
  // Drive to scale and score here
}

}  // namespace autonomous
}  // namespace c2018
