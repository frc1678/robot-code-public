#include "c2018/autonomous/scale_only.h"

#include "muan/units/units.h"

namespace c2018 {
namespace autonomous {

using muan::units::deg;
using frc971::control_loops::drivetrain::Gear::kLowGear;
using frc971::control_loops::drivetrain::Gear::kHighGear;

void ScaleOnly::LeftScale() {
  // XL - Scale is left
  LOG(INFO, "Running LEFT SCALE ONLY auto");
  double scale_x = 5.0;

  // Start drive to left scale
  StartDrivePath(0.0, -5.0, 0.0, -1);
  WaitUntilDriveComplete();
  StartDrivePath(0.5, -5.5, M_PI * 0.5, -1);
  WaitUntilDriveComplete();
  StartDrivePath(5.0, -6.0, 0.0, -1);
  WaitUntilDriveComplete();
  StartDrivePath(scale_x, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();

  // Score reverse here

  // Drive to next cube
  StartDrivePath(4.5, kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Intake cube here

  // Drive back to scale
  StartDrivePath(scale_x, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();

  // Score reverse scale here

  // Drive to 2nd cube
  StartDrivePath(3.25, kCubeXFromReverseWall, 0.0, 1);
  WaitUntilDriveComplete();

  // Drive back to scale
  StartDrivePath(scale_x, kScaleXFromReverseWall, 0.0, -1);
  WaitUntilDriveComplete();
}

void ScaleOnly::RightScale() {
  // XR - Scale is right
  LOG(INFO, "Running RIGHT SCALE ONLY auto");
  double scale_y = -1.2;

  // Drive to scale
  StartDrivePath(kScaleXFromReverseWall, scale_y + 0.1, 30 * deg, -1, kHighGear);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDrivetrainNear(kScaleXFromReverseWall, scale_y + 0.1, 0.4);
  Score();
  WaitUntilDriveComplete();

  // Intake next cube
  IntakeGround();
  StartDrivePath(kCubeXFromReverseWall, -1.27, -15 * deg, 1, kLowGear);
  WaitForCube();
  StopIntakeGround();

  // Drive back to scale
  StartDrivePath(kScaleXFromReverseWall, scale_y, 20 * deg, -1, kLowGear);
  Wait(50);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();
  Score();
  Wait(100);

  // Intake 2nd cube
  IntakeGround();
  StartDrivePath(kCubeXFromReverseWall, -2.12, -20 * deg, 1, kLowGear);
  WaitForCube();
  StopIntakeGround();

  // Drive back to scale
  StartDrivePath(kScaleXFromReverseWall, scale_y, 20 * deg, -1, kLowGear);
  Wait(50);
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  WaitUntilDriveComplete();

  // Scale score reverse here
  Score();
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
}

}  // namespace autonomous
}  // namespace c2018
