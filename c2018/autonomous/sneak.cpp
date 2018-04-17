#include "c2018/autonomous/sneak.h"

namespace c2018 {
namespace autonomous {

using muan::units::deg;
using frc971::control_loops::drivetrain::Gear::kLowGear;
using frc971::control_loops::drivetrain::Gear::kHighGear;

constexpr double kStartY = 3.0;

void Sneak::SneakLeft() {
  SetFieldPosition(0.0, kStartY, 180 * deg);
  // XL - Scale is right
  LOG(INFO, "Running LEFT SCALE ONLY auto");
  double scale_y = 2.1;

  // Drive to scale
  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_SHOOT);
  max_forward_acceleration_ = 9.0;
  StartDrivePath(kScaleX + 0.25, scale_y, 150 * deg, -1, kHighGear, 0, 0, 11);
  WaitUntilDrivetrainNear(kScaleX + 0.25, scale_y, 0.2);
  Score(true);
  WaitUntilDriveComplete();

  // Drive back to pyramid
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  StartDrivePath(1.8, 0.5, 315 * deg, 1, kHighGear, 0.0, 1.0);
  max_forward_acceleration_ = 3.0;
  Wait(300);
  IntakeGround();
  WaitForCube();
  WaitUntilDriveComplete();

  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  StartDrivePath(8.0, 3.3, 180 * deg, -1, kHighGear);
  WaitUntilDriveComplete();

  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_HIGH_FORWARD);

  StartDriveAtAngle(0.0, 90 * deg, 0, kLowGear);
  WaitUntilDriveComplete();

  StartDriveAtAngle(0.4, 90 * deg, 1, kLowGear);
  WaitUntilDriveComplete();

  DropScore();
  Wait(100);
  StartDriveAtAngle(-0.5, 150 * deg, 0, kLowGear);
  Wait(100);
  MoveTo(c2018::score_subsystem::ScoreGoal::INTAKE_0);
}

void Sneak::SneakRight() {
  SetFieldPosition(0.0, kStartY, 180 * deg);

  // XR - Scale is right
  LOG(INFO, "Running RIGHT SNEAK ONLY auto");

  StartDrivePath(0.8, -3.0, 90 * deg, -1);
  MoveTo(c2018::score_subsystem::ScoreGoal::STOW);
  WaitUntilDrivetrainNear(0.8, -3.0, 3.0);

  StartDrivePath(4.0, -3.6, 180 * deg, -1, kHighGear, 0, 0.5);
  WaitUntilDrivetrainNear(4.0, -3.6, 1.0);

  StartDrivePath(8.0, -3.5, 190 * deg, -1, kHighGear, 0, -0.5);
  WaitUntilDriveComplete();

  StartDriveAtAngle(0.0, -90 * deg, 0, kLowGear);
  WaitUntilDriveComplete();

  StartDriveAtAngle(-0.3, -90 * deg, 0, kLowGear);

  MoveTo(c2018::score_subsystem::ScoreGoal::SCALE_HIGH_FORWARD);
  Wait(300);

  StartDriveAtAngle(0.7, -90 * deg, 0, kLowGear);
  WaitUntilDriveComplete();

  DropScore();
  Wait(100);
  StartDriveAtAngle(-0.5, -150 * deg, 0, kLowGear);
  Wait(100);
  MoveTo(c2018::score_subsystem::ScoreGoal::INTAKE_0);
}

}  // namespace autonomous
}  // namespace c2018
