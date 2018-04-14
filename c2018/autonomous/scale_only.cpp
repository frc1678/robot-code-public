#include "c2018/autonomous/scale_only.h"

#include "muan/units/units.h"

namespace c2018 {
namespace autonomous {

using muan::units::deg;
using frc971::control_loops::drivetrain::Gear::kLowGear;
using frc971::control_loops::drivetrain::Gear::kHighGear;

constexpr double kStartY = 3.0;

void ScaleOnly::RightScale() {
  SetFieldPosition(0.0, kStartY, 180 * deg);
  // XR - Scale is right
  LOG(INFO, "Running RIGHT SCALE ONLY auto");
  double scale_y = -2.1;

  // Start drive to left scale
  StartDriveAtAngle(-4.2, 0.0, -2.0);
  Wait(50);
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  WaitUntilDriveComplete();

  StartDrivePath(5.5, 0.0, 90 * deg, -1);
  WaitUntilDrivetrainNear(5.5, 0.0, 2.0);

  StartDriveAtAngle(-3.0, -90 * deg, -2.0);
  Wait(250);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();

  StartDrivePath(kScaleX + 0.5, scale_y, 200 * deg, -1);
  WaitUntilDriveComplete();
  Score(false);
  Wait(100);

  // Drive to next cube
  IntakeGround();
  Wait(100);
  IntakeOpen();
  Wait(50);
  StartDrivePath(kCubeX + 0.2, -2.05, 175 * deg, 1, kLowGear);
  WaitUntilDrivetrainNear(kCubeX + 0.2, -2.05, 0.2);

  // Intake cube
  IntakeClose();
  WaitForCube();
  WaitUntilDriveComplete();
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);

  // Drive back to scale
  StartDrivePath(kScaleX + 0.6, scale_y + 0.3, 220 * deg, -1, kLowGear);
  WaitUntilDriveComplete();
  // Score backwards on the scale
  Score(false);
  Wait(100);
  IntakeGround();

  // Drive to 2nd cube
  StartDrivePath(kCubeX + 0.3, -1.2, 135 * deg, 1, kLowGear, 0.0, 0.3);
  WaitUntilDrivetrainNear(kCubeX + 0.1, -1.4, 0.2);
  WaitForCube();
  WaitUntilDriveComplete();

  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);

  // Drive back to scale
  StartDrivePath(kScaleX + 0.1, scale_y + 0.1, 220 * deg, -1, kLowGear);
  WaitUntilDriveComplete();
  Score(false);
  Wait(100);
  IntakeGround();
  StartDrivePath(kCubeX + 0.1, -0.3, 135 * deg, 1, kLowGear);
  WaitUntilDrivetrainNear(kCubeX + 0.1, -0.3, 0.2);
  WaitForCube();
}

void ScaleOnly::LeftScale() {
  SetFieldPosition(0.0, kStartY, 180 * deg);
  // XL - Scale is right
  LOG(INFO, "Running LEFT SCALE ONLY auto");
  double scale_y = 2.1;

  // Drive to scale
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  StartDrivePath(kScaleX + 0.45, scale_y, 150 * deg, -1, kHighGear);
  WaitUntilDrivetrainNear(kScaleX + 0.45, scale_y, 0.2);
  Score(false);
  Wait(100);
  WaitUntilDriveComplete();

  // Intake next cube
  IntakeGround();
  IntakeOpen();
  Wait(50);
  StartDrivePath(kCubeX + 0.1, 1.85, 195 * deg, 1, kLowGear);
  WaitUntilDrivetrainNear(kCubeX + 0.1, 1.85, 0.2);
  IntakeClose();
  WaitUntilDriveComplete();
  if (!WaitForCubeOrTimeout(300)) {
    LOG(WARNING, "Stuck on a cube! Backing up anyway!");
  }

  // Drive back to scale
  StartDrivePath(kScaleX + 0.5, scale_y, 150 * deg, -1, kLowGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();
  Score(true);
  Wait(50);

  // Intake 2nd cube
  IntakeGround();
  Wait(100);
  StartDrivePath(kCubeX - 0.1, 1.25, 225 * deg, 1, kLowGear, 0.0, 0.4);
  WaitForCube();
  WaitUntilDriveComplete();

  // Drive back to scale
  StartDrivePath(kScaleX + 0.5, scale_y, 135 * deg, -1, kLowGear);
  Wait(50);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();

  // Scale score reverse here
  Score(true);
  Wait(100);

  IntakeGround();
  Wait(100);
  StartDrivePath(kCubeX - 0.15, 0.6, 225 * deg, 1, kLowGear, 0.6);
  WaitForCube();
}

}  // namespace autonomous
}  // namespace c2018
