#include "c2019/commands/cargo_rocket.h"
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {

using c2019::limelight::LimelightStatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

void CargoRocket::RightCargoRocket() {
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = 0.0;
  max_lin_ = 3.0;
  max_acc_ = 3.0;
  SetFieldPosition(1.8, -1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running RIGHT CARGO ROCKET auto");

  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  GoTo(superstructure::REZERO);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  StartDrivePath(5.3, -0.2, 0 * (M_PI / 180), 1, true);
  WaitUntilDrivetrainNear(5.3, -0.2, .3);
  bool success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(40);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      5.2, -.2, (drive_status->estimated_heading() - init_gyro) + init_theta);

  max_lin_ = 3.0;
  max_acc_ = 3.0;

  final_vel_ = 1.5;
  StartDrivePath(1.1, -3.5, 0.0, -1, true, true, 0.35);

  WaitUntilDrivetrainNear(3.7, -.9, .6);
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS, superstructure::INTAKE_HATCH);

  final_vel_ = 0.0;
  WaitUntilDriveComplete();
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVisionBackwards();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, -3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7.6, -2.8, -220 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(6.2, -2.3, .6);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  StartPointTurn(90 * (M_PI / 180.));
  WaitUntilDriveComplete();
  success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(50);
  ExitAutonomous();  // bye
  return;
}

void CargoRocket::LeftCargoRocket() {
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = 0.0;
  max_lin_ = 3.0;
  max_acc_ = 3.0;
  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running LEFT CARGO ROCKET auto");

  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  GoTo(superstructure::REZERO);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  StartDrivePath(5.3, 0.2, 0 * (M_PI / 180), 1, true);
  WaitUntilDrivetrainNear(5.3, 0.2, .3);
  bool success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(40);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      5.2, .2, (drive_status->estimated_heading() - init_gyro) + init_theta);

  max_lin_ = 3.0;
  max_acc_ = 3.0;

  final_vel_ = 1.5;
  StartDrivePath(1.1, 3.5, 0.0, -1, true, true, 0.35);

  WaitUntilDrivetrainNear(3.7, .9, .6);
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS, superstructure::INTAKE_HATCH);

  final_vel_ = 0.0;
  WaitUntilDriveComplete();
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVisionBackwards();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7.6, 2.8, 220 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(6.2, 2.3, .6);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  StartPointTurn(-90 * (M_PI / 180.));
  WaitUntilDriveComplete();
  success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(50);
  ExitAutonomous();  // bye
  return;
}

}  // namespace commands
}  // namespace c2019
