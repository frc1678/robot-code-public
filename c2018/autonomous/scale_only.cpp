#include "c2018/autonomous/scale_only.h"

#include "muan/units/units.h"

namespace c2018 {
namespace autonomous {

using muan::units::deg;
using frc971::control_loops::drivetrain::Gear::kLowGear;
using frc971::control_loops::drivetrain::Gear::kHighGear;

constexpr double kStartY = -3.0;

void ScaleOnly::LeftScale() {
  SetFieldPosition(0.0, kStartY, 180 * deg);
  // XL - Scale is left
  LOG(INFO, "Running LEFT SCALE ONLY auto");
  double scale_y = 2.1;

  // Start drive to left scale
  StartDriveAtAngle(-4.0, 0.0, -2.0);
  Wait(50);
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  WaitUntilDriveComplete();

  StartDrivePath(5.5, 0.0, 270 * deg, -1);
  WaitUntilDrivetrainNear(5.0, 0.0, 2.0);

  StartDriveAtAngle(-3.0, 90 * deg, -2.0);
  Wait(150);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();

  StartDrivePath(kScaleX + 0.8, scale_y, 160 * deg, -1);
  WaitUntilDriveComplete();
  Score();

  // Score reverse here
  Wait(100);

  // Drive to next cube
  IntakeGround();
  IntakeOpen();
  Wait(50);
  StartDrivePath(kCubeX + 0.5, 1.93, 180 * deg, 1, kLowGear, 0.0, 0.3);
  WaitUntilDriveComplete();

  // Intake cube
  IntakeClose();
  WaitForCube();
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);

  // Drive back to scale
  StartDrivePath(kScaleX + 0.8, scale_y, 160 * deg, -1, kLowGear);
  WaitUntilDriveComplete();

  // Score backwards on the scale
  Score();
  Wait(100);
  IntakeGround();
  IntakeOpen();

  // Drive to 2nd cube
  StartDrivePath(kCubeX + 0.5, 1.18, 180 * deg, 1, kLowGear, 0.3, 0.3);
  WaitUntilDriveComplete();
  IntakeClose();
  WaitForCube();

  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);

  // Drive back to scale
  StartDrivePath(kScaleX + 0.8, scale_y, 160 * deg, -1, kLowGear);
  WaitUntilDriveComplete();
  Score();
  Wait(100);
  IntakeGround();
}

void ScaleOnly::RightScale() {
  SetFieldPosition(0.0, kStartY, 180 * deg);
  // XR - Scale is right
  LOG(INFO, "Running RIGHT SCALE ONLY auto");
  double scale_y = -2.1;

  // Drive to scale
  StartDrivePath(kScaleX + 0.3, scale_y, 210 * deg, -1, kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();
  Wait(50);
  Score();
  Wait(100);

  // Intake next cube
  IntakeGround();
  IntakeOpen();
  Wait(50);
  StartDrivePath(kCubeX + 0.2, -1.8, 165 * deg, 1, kLowGear);
  WaitUntilDriveComplete();
  IntakeClose();
  WaitForCube();

  // Drive back to scale
  StartDrivePath(kScaleX + 0.15, scale_y, 210 * deg, -1, kLowGear);
  Wait(50);
  StopIntakeGround();
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  Wait(100);
  WaitUntilDriveComplete();
  Score();
  Wait(50);

  // Intake 2nd cube
  IntakeGround();
  IntakeOpen();
  Wait(100);
  StartDrivePath(kCubeX + 0.1, -0.7, 160 * deg, 1, kLowGear, 0.0, 0.3);
  WaitUntilDriveComplete();
  IntakeClose();
  WaitForCube();

  // Drive back to scale
  StartDrivePath(kScaleX + 0.3, scale_y, 225 * deg, -1, kLowGear);
  Wait(50);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();

  // Scale score reverse here
  Score();
  Wait(100);
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
}

}  // namespace autonomous
}  // namespace c2018
