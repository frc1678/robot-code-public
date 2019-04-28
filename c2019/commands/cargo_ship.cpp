#include "c2019/commands/cargo_ship.h"
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {

using c2019::limelight::LimelightStatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

void CargoShip::RightFrontCargoShip() {
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = 0.0;
  max_lin_ = 3.0;
  max_acc_ = 3.0;
  SetFieldPosition(1.8, -1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running RIGHT FRONT CARGO SHIP auto");

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
  StartDrivePath(1.1, -3.2, 0.0, -1, true, true, 0.35);

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

  Wait(10);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, -3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  final_vel_ = 2.5;
  StartDrivePath(5.0, -2.4, 0.0, 1, true);
  WaitUntilDriveComplete();
  final_vel_ = 1.0;
  StartDrivePath(7.95, -1.5, 75 * (M_PI / 180.), 1, true, true);
  WaitUntilDriveComplete();

  success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(50);

  final_vel_ = 0.0;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      7.65, -1.1, (drive_status->estimated_heading() - init_gyro) + init_theta);

  max_lin_ = 2.0;
  max_acc_ = 2.0;
  StartDrivePath(7.1, -2.3, 90. * (M_PI / 180.), -1, true);
  Wait(40);
  GoTo(superstructure::CARGO_AUTO, superstructure::INTAKE_CARGO);
  WaitUntilDriveComplete();

  StartDrivePath(7.1, -1.37, 90 * (M_PI / 180.), 1, true, true);
  WaitUntilDriveComplete();
  ExitAutonomous();  // bye
}

void CargoShip::LeftFrontCargoShip() {
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = 0.0;
  max_lin_ = 3.0;
  max_acc_ = 3.0;
  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running LEFT FRONT CARGO SHIP auto");

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

  Wait(10);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  final_vel_ = 2.5;
  StartDrivePath(5.0, 2.4, 0.0, 1, true);
  WaitUntilDriveComplete();
  final_vel_ = 1.0;
  StartDrivePath(7.95, 1.5, -75 * (M_PI / 180.), 1, true, true);
  WaitUntilDriveComplete();

  success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(50);

  final_vel_ = 0.0;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      7.65, 1.1, (drive_status->estimated_heading() - init_gyro) + init_theta);

  max_lin_ = 2.0;
  max_acc_ = 2.0;
  StartDrivePath(7.1, 2.3, -90. * (M_PI / 180.), -1, true);
  Wait(40);
  GoTo(superstructure::CARGO_AUTO, superstructure::INTAKE_CARGO);
  WaitUntilDriveComplete();

  StartDrivePath(7.1, 1.37, -90 * (M_PI / 180.), 1, true, true);
  WaitUntilDriveComplete();
  ExitAutonomous();  // bye
}

void CargoShip::RightSideCargoShip() {
  EnterAutonomous();
  double init_theta = 0.0;
  max_lin_ = 3.0;
  max_acc_ = 3.0;
  SetFieldPosition(1.8, -1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running RIGHT CARGO SHIP auto");

  // score hatch on left side, 2, of the CS
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  GoTo(superstructure::REZERO);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  final_vel_ = 0.5;
  StartDrivePath(7., -1.3, 80 * (M_PI / 180), 1, true);

  WaitUntilDrivetrainNear(7., -1.3, .2);
  bool success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(50);

  // Go pick up hatch from right loading station
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(7.15, -1,
                   drive_status->estimated_heading());  // At right 2nd CS

  final_vel_ = 2.5;
  StartDrivePath(5.0, -2.4, 0.0, -1, true, true);
  WaitUntilDriveComplete();
  final_vel_ = 0.0;
  StartDrivePath(.4, -3.4, 0.0, -1, true, true);
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(.4, -3.4, .6);
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVisionBackwards();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  Wait(10);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, -3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  final_vel_ = 2.5;
  StartDrivePath(5.0, -2.4, 0.0, 1, true);
  WaitUntilDriveComplete();
  final_vel_ = 1.0;
  StartDrivePath(7.65, -1.5, 80 * (M_PI / 180.), 1, true, true);
  WaitUntilDriveComplete();

  success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      7.65, -1.1, (drive_status->estimated_heading() - init_gyro) + init_theta);
  max_lin_ = 3.5;
  max_acc_ = 4.0;

  final_vel_ = 3.0;
  StartDrivePath(5.0, -2.4, 0.0, -1, true);
  WaitUntilDriveComplete();
  final_vel_ = 0.0;
  StartDrivePath(.7, -3.4, 0.0, -1, true, true);
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS, superstructure::INTAKE_HATCH);
  WaitUntilDriveComplete();

  ExitAutonomous();
}

void CargoShip::LeftSideCargoShip() {
  EnterAutonomous();
  double init_theta = 0.0;
  max_lin_ = 3.0;
  max_acc_ = 3.0;
  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running LEFT CARGO SHIP auto");

  // score hatch on left side, 2, of the CS
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  GoTo(superstructure::REZERO);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  final_vel_ = 0.5;
  StartDrivePath(7., 1.3, -80 * (M_PI / 180), 1, true);

  WaitUntilDrivetrainNear(7., 1.3, .2);
  bool success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(50);

  // Go pick up hatch from right loading station
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(7.15, 1,
                   drive_status->estimated_heading());  // At right 2nd CS

  final_vel_ = 2.5;
  StartDrivePath(5.0, 2.4, 0.0, -1, true, true);
  WaitUntilDriveComplete();
  final_vel_ = 0.0;
  StartDrivePath(.4, 3.4, 0.0, -1, true, true);
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(.4, 3.4, .6);
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVisionBackwards();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  Wait(10);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  final_vel_ = 2.5;
  StartDrivePath(5.0, 2.4, 0.0, 1, true);
  WaitUntilDriveComplete();
  final_vel_ = 1.0;
  StartDrivePath(7.65, 1.5, -80 * (M_PI / 180.), 1, true, true);
  WaitUntilDriveComplete();

  success = StartDriveVision();
  if (!success) {
    LOG(WARNING, "vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      7.65, 1.1, (drive_status->estimated_heading() - init_gyro) + init_theta);
  max_lin_ = 3.5;
  max_acc_ = 4.0;

  final_vel_ = 3.0;
  StartDrivePath(5.0, 2.4, 0.0, -1, true);
  WaitUntilDriveComplete();
  final_vel_ = 0.0;
  StartDrivePath(.7, 3.4, 0.0, -1, true, true);
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS, superstructure::INTAKE_HATCH);
  WaitUntilDriveComplete();

  ExitAutonomous();
}

}  // namespace commands
}  // namespace c2019
