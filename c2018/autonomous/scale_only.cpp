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
  double scale_y = 2.0;

  // Start drive to left scale
  StartDriveAtAngle(-4.0, 0.0, -2.0);
  WaitUntilDriveComplete();

  StartDrivePath(5.0, 0.0, 270 * deg, -1);
  WaitUntilDrivetrainNear(5.0, 0.0, 2.0);

  StartDriveAtAngle(-3.0, 90 * deg, -2.0);
  Wait(150);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();

  StartDrivePath(kScaleX + 0.2, scale_y, 160 * deg, -1);
  WaitUntilDrivetrainNear(kScaleX + 0.2, scale_y, 0.5);
  Score();
  WaitUntilDriveComplete();

  // Score reverse here
  Wait(100);

  // Drive to next cube
  IntakeGround();
  Wait(50);
  StartDrivePath(kCubeX + 0.2, 1.93, 180 * deg, 1, kLowGear, 0.0, 0.3);

  // Intake cube
  WaitForCube();
  StopIntakeGround();
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);

  // Drive back to scale
  StartDrivePath(kScaleX + 0.2, scale_y, 160 * deg, -1, kLowGear);
  WaitUntilDrivetrainNear(kScaleX + 0.2, scale_y, 0.3);
  Score();
  WaitUntilDriveComplete();

  // Score backwards on the scale
  Score();
  Wait(100);
  IntakeGround();

  // Drive to 2nd cube
  StartDrivePath(kCubeX + 0.2, 0.88, 180 * deg, 1, kLowGear, 0.3, 0.3);
  WaitForCube();
  StopIntakeGround();

  // Drive back to scale
  StartDrivePath(kScaleX + 0.2, scale_y, 160 * deg, -1, kLowGear);
  WaitUntilDrivetrainNear(kScaleX + 0.2, scale_y, 0.3);
  Score();
  WaitUntilDriveComplete();
}

void ScaleOnly::RightScale() {
  SetFieldPosition(0.0, kStartY, 180 * deg);
  // XR - Scale is right
  LOG(INFO, "Running RIGHT SCALE ONLY auto");
  double scale_y = -1.9;

  // Drive to scale
  StartDrivePath(kScaleX, scale_y, 210 * deg, -1, kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDrivetrainNear(kScaleX, scale_y, 0.4);
  Score();
  WaitUntilDriveComplete();

  // Intake next cube
  IntakeGround();
  StartDrivePath(kCubeX, -1.73, 170 * deg, 1, kLowGear);
  WaitForCube();
  StopIntakeGround();

  // Drive back to scale
  StartDrivePath(kScaleX, scale_y, 200 * deg, -1, kLowGear);
  Wait(50);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();
  Score();
  Wait(100);

  // Intake 2nd cube
  IntakeGround();
  StartDrivePath(kCubeX, -0.83, 180 * deg, 1, kLowGear);
  WaitForCube();
  StopIntakeGround();

  // Drive back to scale
  StartDrivePath(kScaleX, scale_y, 200 * deg, -1, kLowGear);
  Wait(50);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();

  // Scale score reverse here
  Score();
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
}

}  // namespace autonomous
}  // namespace c2018
