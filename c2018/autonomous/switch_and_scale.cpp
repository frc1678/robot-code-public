#include "c2018/autonomous/switch_and_scale.h"

namespace c2018 {
namespace autonomous {

using frc971::control_loops::drivetrain::Gear;
using muan::units::deg;

void SwitchAndScale::RightRightSwitch() {
  LOG(INFO, "Running RIGHT RIGHT SWITCH DRIVE auto");
  SetFieldPosition(0.0, -0.15, 180 * deg);
  // Drive to switch 1st time
  StartDrivePath(2.65, -1.5, 160 * deg, -1, Gear::kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_LOW_REVERSE);
  WaitUntilDrivetrainNear(2.65, -1.5, 0.2);
  Score(false);
  WaitUntilDriveComplete();
  IntakeGround();
  // Drive into cube
  StartDrivePath(1.75, -0.5, 45 * deg, 1, Gear::kLowGear, 0.0, 0.4);
  WaitForCube();
  WaitUntilDriveComplete();
  StopIntakeGround();
  // Drive to switch
  StartDrivePath(2.85, -1.4, 180 * deg, -1, Gear::kLowGear, 0.0, 0.6, 8.0);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_LOW_REVERSE);
  WaitUntilDrivetrainNear(2.65, -1.4, 0.2);
  Score(false);
  WaitUntilDriveComplete();

  MoveTo(c2018::score_subsystem::ScoreGoal::INTAKE_1);
  IntakeOpen();

  // Drive into  2nd cube
  StartDrivePath(2.25, -0.1, 0 * deg, 1, Gear::kLowGear, 0.0, 0.4);
  WaitUntilDrivetrainNear(2.25, -0.1, 0.2);
  IntakeClose();
  WaitForCube();
  WaitUntilDriveComplete();

  // Turn off of pyramid
  StartDriveAtAngle(-0.2, -90 * deg, 1.5);
  StopIntakeGround();
  WaitUntilDriveComplete();

  // Drive to right side
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  StartDrivePath(3.5, -3.65, 180 * deg, -1, Gear::kHighGear, 0.0, 0.7);
  WaitUntilDrivetrainNear(3.5, -3.65, 1.5);
  StartDriveAtAngle(-6.5, 0.0);
  WaitUntilDriveComplete();
}

void SwitchAndScale::RightLeftSwitch() {
  LOG(INFO, "Running RIGHT LEFT SWITCH DRIVE auto");
  SetFieldPosition(0.0, -0.15, 180 * deg);
  // Drive to switch 1st time
  StartDrivePath(2.65, -1.5, 160 * deg, -1, Gear::kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_LOW_REVERSE);
  WaitUntilDrivetrainNear(2.65, -1.5, 0.2);
  Score(false);
  WaitUntilDriveComplete();
  IntakeGround();
  // Drive into cube
  StartDrivePath(1.75, -0.5, 45 * deg, 1, Gear::kLowGear, 0.0, 0.4);
  WaitForCube();
  WaitUntilDriveComplete();
  StopIntakeGround();
  // Drive to switch
  StartDrivePath(2.85, -1.4, 180 * deg, -1, Gear::kLowGear, 0.0, 0.6, 8.0);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_LOW_REVERSE);
  WaitUntilDrivetrainNear(2.65, -1.4, 0.2);
  Score(false);
  WaitUntilDriveComplete();

  MoveTo(c2018::score_subsystem::ScoreGoal::INTAKE_1);
  IntakeOpen();

  // Drive into  2nd cube
  StartDrivePath(2.25, -0.1, 0 * deg, 1, Gear::kLowGear, 0.0, 0.4);
  WaitUntilDrivetrainNear(2.25, -0.1, 0.2);
  IntakeClose();
  WaitForCube();
  WaitUntilDriveComplete();

  // Turn off of pyramid
  StartDriveAtAngle(-0.2, -90 * deg, 1.5);
  StopIntakeGround();
  WaitUntilDriveComplete();

  // Drive to right side
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  StartDrivePath(3.5, 3.65, 180 * deg, -1, Gear::kHighGear, 0.0, 0.7);
  WaitUntilDrivetrainNear(3.5, 3.65, 1.5);
  StartDriveAtAngle(-6.5, 0.0);
  WaitUntilDriveComplete();
}

void SwitchAndScale::LeftRightSwitch() {
  LOG(INFO, "Running LEFT RIGHT SWITCH DRIVE auto");
  SetFieldPosition(0.0, -0.15, 180 * deg);
  // Drive to switch 1st time
  StartDrivePath(2.65, 1.5, 200 * deg, -1, Gear::kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_LOW_REVERSE);
  WaitUntilDrivetrainNear(2.65, 1.5, 0.2);
  Score(false);
  WaitUntilDriveComplete();
  IntakeGround();
  // Drive into cube
  StartDrivePath(1.75, 0.55, 315 * deg, 1, Gear::kLowGear, 0.0, 0.4);
  WaitForCube();
  WaitUntilDriveComplete();
  StopIntakeGround();
  // Drive to switch
  StartDrivePath(2.85, 1.4, 180 * deg, -1, Gear::kLowGear, 0.0, 0.6, 8.0);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_LOW_REVERSE);
  WaitUntilDrivetrainNear(2.65, 1.4, 0.2);
  Score(false);
  WaitUntilDriveComplete();

  MoveTo(c2018::score_subsystem::ScoreGoal::INTAKE_1);
  IntakeOpen();

  // Drive into  2nd cube
  StartDrivePath(2.25, 0.4, 360 * deg, 1, Gear::kLowGear, 0.0, 0.4);
  WaitUntilDrivetrainNear(2.25, 0.4, 0.2);
  IntakeClose();
  WaitForCube();
  WaitUntilDriveComplete();

  // Turn off of pyramid
  StartDriveAtAngle(-0.2, -90 * deg, 1.5);
  StopIntakeGround();
  WaitUntilDriveComplete();

  // Drive to right side
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  StartDrivePath(3.5, -3.65, 180 * deg, -1, Gear::kHighGear, 0.0, 0.7);
  WaitUntilDrivetrainNear(3.5, -3.65, 1.5);
  StartDriveAtAngle(-6.5, 0.0);
  WaitUntilDriveComplete();
}

void SwitchAndScale::LeftLeftSwitch() {
  LOG(INFO, "Running LEFT LEFT SWITCH DRIVE auto");
  SetFieldPosition(0.0, -0.15, 180 * deg);
  // Drive to switch 1st time
  StartDrivePath(2.65, 1.5, 200 * deg, -1, Gear::kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_LOW_REVERSE);
  WaitUntilDrivetrainNear(2.65, 1.5, 0.2);
  Score(false);
  WaitUntilDriveComplete();
  IntakeGround();
  // Drive into cube
  StartDrivePath(1.75, 0.55, 315 * deg, 1, Gear::kLowGear, 0.0, 0.4);
  WaitForCube();
  WaitUntilDriveComplete();
  StopIntakeGround();
  // Drive to switch
  StartDrivePath(2.85, 1.4, 180 * deg, -1, Gear::kLowGear, 0.0, 0.6, 8.0);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_LOW_REVERSE);
  WaitUntilDrivetrainNear(2.65, 1.4, 0.2);
  Score(false);
  WaitUntilDriveComplete();

  MoveTo(c2018::score_subsystem::ScoreGoal::INTAKE_1);
  IntakeOpen();

  // Drive into  2nd cube
  StartDrivePath(2.25, 0.4, 360 * deg, 1, Gear::kLowGear, 0.0, 0.4);
  WaitUntilDrivetrainNear(2.25, 0.4, 0.2);
  IntakeClose();
  WaitForCube();
  WaitUntilDriveComplete();

  // Turn off of pyramid
  StartDriveAtAngle(-0.2, 90 * deg, 1.5);
  StopIntakeGround();
  WaitUntilDriveComplete();

  // Drive to right side
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  StartDrivePath(3.5, 3.65, 180 * deg, -1, Gear::kHighGear, 0.0, 0.7);
  WaitUntilDrivetrainNear(3.5, 3.65, 1.5);
  StartDriveAtAngle(-6.5, 0.0);
  WaitUntilDriveComplete();
}

}  // namespace autonomous
}  // namespace c2018
