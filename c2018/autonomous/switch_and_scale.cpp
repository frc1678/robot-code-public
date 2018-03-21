#include "c2018/autonomous/switch_and_scale.h"

#include <cmath>

namespace c2018 {
namespace autonomous {

using muan::units::deg;
using frc971::control_loops::drivetrain::Gear::kLowGear;
using frc971::control_loops::drivetrain::Gear::kHighGear;

void SwitchAndScale::LeftSwitchLeftScale() {
  SetFieldPosition(0.0, -0.3, M_PI);
  // Switch is left, scale is left
  LOG(INFO, "Running LEFT SWITCH LEFT SCALE auto");

  // Drive to backside of switch
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  StartDrivePath(kCubeX + 0.8, 3.0, M_PI, -1, kHighGear, 0.0, 2.0);
  WaitUntilDriveComplete();

  // Drive into cube
  StartDrivePath(kCubeX + 0.4, 2.1, 230 * deg, 1, kLowGear);
  WaitUntilDriveComplete();
  // Score here
  Score();
  Wait(100);

  // Drive back & lower intake
  StartDriveRelative(-0.7, 0.0);
  Wait(100);
  IntakeOpen();
  IntakeGround();
  WaitUntilDriveComplete();
  Wait(100);

  // Get the cube
  StartDriveRelative(0.7, 0.0);
  WaitForCube();
  IntakeClose();
  StopIntakeGround();
  // Drive to scale
  StartDriveRelative(-2.0, -40 * deg);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_HIGH_REVERSE);
  WaitUntilDriveComplete();

  Score(false);
  Wait(100);

  IntakeGround();
}

void SwitchAndScale::RightSwitchRightScale() {
  SetFieldPosition(0.0, -0.3, M_PI);
  // LL - Switch is left, scale is left
  LOG(INFO, "Running RIGHT SWITCH RIGHT SCALE auto");

  // Drive to backside of switch
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  StartDrivePath(4.0, -3.75, M_PI, -1, kHighGear, 0.0, 1.0);
  WaitUntilDriveComplete();
  StartDriveRelative(0.0, -1.1);
  WaitUntilDriveComplete();

  // Score here
  Score();
  Wait(100);

  // Drive to cubes in PZ
  IntakeGround();
  StartDrivePath(kCubeX + 1.3, -2.6, 160 * deg, -1);
  WaitUntilDriveComplete();

  // Get the cube
  StartDrivePath(kCubeX + 0.2, -2.28, 175 * deg, 1);
  WaitForCube();
  StopIntakeGround();

  // Drive to scale
  StartDrivePath(kScaleX + 0.2, -2.1, 190 * deg, -1);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  while (!IsDrivetrainNear(kScaleX + 0.2, -2.5, 0.5) && !IsDriveComplete()) {
    loop_.SleepUntilNext();
  }

  Score(false);
  Wait(100);
}

void SwitchAndScale::RightSwitchLeftScale() {
  SetFieldPosition(0.0, -0.3, M_PI);
  // LL - Switch is left, scale is left
  LOG(INFO, "Running RIGHT SWITCH LEFT SCALE auto");

  // Drive to backside of switch
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  StartDrivePath(kCubeX + 1.3, -3.3, M_PI, -1, kHighGear, 0.0, 2.0);
  WaitUntilDriveComplete();

  // Drive into cube
  StartDrivePath(kCubeX + 0.35, -2.2, 150 * deg, 1, kHighGear);
  WaitUntilDriveComplete();
  // Score here
  Score();
  Wait(100);

  // Drive back & lower intake
  StartDriveRelative(-0.7, 0.0);
  Wait(150);
  IntakeOpen();
  IntakeGround();
  WaitUntilDriveComplete();
  Wait(100);

  // Get the cube
  StartDriveRelative(0.6, 0.0);
  Wait(150);
  IntakeClose();
  WaitForCube();

  // Drive to scale
  StartDrivePath(kScaleX - 0.3, 2.7, 160 * deg, -1, kHighGear, -3.0, 0.3);
  WaitUntilDrivetrainNear(kCubeX + 0.5, 2.7, 2.0);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();

  Wait(100);
  Score();
  Wait(100);

  IntakeGround();
}

void SwitchAndScale::LeftSwitchRightScale() {
  SetFieldPosition(0.0, -0.3, M_PI);
  // Switch is left, scale is right
  LOG(INFO, "Running LEFT SWITCH RIGHT SCALE auto");

  // Drive to backside of switch
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  StartDrivePath(kCubeX + 0.8, 3.0, M_PI, -1, kHighGear, 0.0, 2.0);
  WaitUntilDriveComplete();

  // Drive into cube
  StartDrivePath(kCubeX + 0.7, 1.8, 215 * deg, 1, kHighGear);
  WaitUntilDriveComplete();
  // Score here
  Score();
  Wait(100);

  // Drive back & lower intake
  StartDrivePath(kCubeX + 0.7, 2.0, 180 * deg, -1);
  Wait(150);
  IntakeOpen();
  IntakeGround();
  WaitUntilDriveComplete();
  Wait(100);

  // Get the cube
  StartDrivePath(kCubeX, 2.0, 180 * deg, 1);
  Wait(150);
  IntakeClose();
  WaitForCube();

  // Drive to scale on right side
  StartDrivePath(kCubeX - 1.5, -2.5, 90 * deg, -1);
  StopIntakeGround();
  WaitUntilDrivetrainNear(5.75, -1.5, 1.0);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  StartDrivePath(kScaleX + 1.7, -2.0, 200 * deg, -1);
  WaitUntilDriveComplete();

  // Score on scale here
  Score();
  Wait(100);

  IntakeGround();
}

}  // namespace autonomous
}  // namespace c2018
