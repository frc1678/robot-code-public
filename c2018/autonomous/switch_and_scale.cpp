#include "c2018/autonomous/switch_and_scale.h"

#include <cmath>

namespace c2018 {
namespace autonomous {

using muan::units::deg;
using frc971::control_loops::drivetrain::Gear::kLowGear;
using frc971::control_loops::drivetrain::Gear::kHighGear;

void SwitchAndScale::LeftSwitchLeftScale() {
  SetFieldPosition(0.0, -0.2, M_PI);
  // LL - Switch is left, scale is left
  LOG(INFO, "Running LEFT SWITCH LEFT SCALE auto");

  // Drive to backside of switch
  MoveTo(c2018::score_subsystem::ScoreGoal::SWITCH);
  StartDrivePath(4, 3.25, M_PI, -1, kHighGear, 0.0, 1.0);
  WaitUntilDriveComplete();
  StartDriveRelative(0.0, 1.0);
  WaitUntilDriveComplete();

  // Score here
  Score();
  Wait(100);

  // Drive to cubes in PZ
  StartDrivePath(kCubeX + 1.0, 2.5, 200 * deg, -1);
  WaitUntilDriveComplete();

  // Get the cube
  IntakeGround();
  StartDrivePath(kCubeX, 2.5, 180 * deg, 1);
  WaitForCube();
  StopIntakeGround();

  // Drive to scale
  StartDrivePath(kScaleX + 0.3, 2.1, 160 * deg, -1);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDrivetrainNear(kScaleX + 0.3, 2.5, 0.5);

  Score();
  Wait(100);
}

void SwitchAndScale::RightSwitchRightScale() {
  SetFieldPosition(0.0, 0.0, M_PI);
  // RR - Switch is right, scale is right
  LOG(INFO, "Running RIGHT SWITCH RIGHT SCALE auto");

  // Initial drive toward backside of switch
  StartDrivePath(-2.5, -2.5, 0.0, -1);
  WaitUntilDriveComplete();
  StartDrivePath(-2., -5., M_PI * 0.25, -1);
  WaitUntilDriveComplete();

  // Score cube in switch here

  // Pick up next cube
  StartDrivePath(-1.25, kCubeX, 0.0, 1);
  WaitUntilDriveComplete();

  // Drive to scale
  StartDrivePath(-1.75, kScaleX, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here

  // Drive to 2nd cube
  StartDrivePath(-0.8, kCubeX, 0.0, 1);
  WaitUntilDriveComplete();

  // Intake 2nd cube here

  // Drive to scale
  StartDrivePath(-1.75, c2018::autonomous::kScaleX, 0.0, -1);
  WaitUntilDriveComplete();

  // Score here
}

void SwitchAndScale::RightSwitchLeftScale() {
  // RL - Switch is right, scale is left
  LOG(INFO, "Running RIGHT SWITCH LEFT SCALE auto");

  // Initial drive to backside of switch
  StartDrivePath(-2.5, -2.5, 0.0, -1);
  WaitUntilDriveComplete();
  StartDrivePath(-2, -5.5, M_PI * -0.25, -1);
  WaitUntilDriveComplete();

  // Score here

  // Drive to next cube
  StartDrivePath(-1.5, kCubeX, 0.0, 1);
  WaitUntilDriveComplete();

  // Intake cube here

  // Drive to scale
  StartDrivePath(2.0, -5.5, M_PI * 0.5, -1);
  WaitUntilDriveComplete();
  StartDrivePath(2.5, kScaleX, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here

  // Drive to 2nd cube
  StartDrivePath(2.25, kCubeX, 0.0, 1);
  WaitUntilDriveComplete();

  // Intake 2nd cube here

  // Drive back to scale
  StartDrivePath(2.5, kScaleX, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here
}

void SwitchAndScale::LeftSwitchRightScale() {
  // Switch is left, scale is right
  LOG(INFO, "Running LEFT SWITCH RIGHT SCALE auto");

  // Drive to backside of switch
  StartDrivePath(3.2, -3, 0.0, -1);
  WaitUntilDriveComplete();
  StartDrivePath(3.2, -5, M_PI * 0.25, -1);
  WaitUntilDriveComplete();

  // Score here

  // Drive to cubes in PZ
  StartDrivePath(2., kCubeX, 0.0, 1);
  WaitUntilDriveComplete();

  // Pick up cube here

  // Drive to scale on right side
  StartDrivePath(5.75, -1.75, M_PI * -0.5, -1);
  WaitUntilDriveComplete();
  StartDrivePath(kScaleX, -1.2, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here

  // Drive to 2nd cube
  StartDrivePath(-1.25, kCubeX, 0.0, 1);
  WaitUntilDriveComplete();

  // Intake cube here

  // Drive to scale again
  StartDrivePath(-1.2, kScaleX, 0.0, -1);
  WaitUntilDriveComplete();

  // Score on scale here
}

}  // namespace autonomous
}  // namespace c2018
