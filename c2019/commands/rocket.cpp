#include "c2019/commands/rocket.h"
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {

using c2019::limelight::LimelightStatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

void Rocket::LeftRocket() {
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = M_PI;

  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  StartDrivePath(7.4, 3.1, 210 * (M_PI / 180.), -1, true);
  GoTo(superstructure::CARGO_GROUND, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(7.4, 3.1, 0.2);
  StartPointTurn(-60 * (M_PI / 180.));
  Wait(50);
  bool success = StartDriveVision();
  if (!success) {
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(25);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      6.4, 3.5, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7.1, 2.1, 150. * (M_PI / 180.), -1, true);

  WaitUntilDriveComplete();
  StartDrivePath(.4, 3.3, M_PI, 1, true, true, 0, 0.5);
  Wait(150);
  GoTo(superstructure::HATCH_SHIP_FORWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(1.4, 3.4, 0.6);
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVision(0.65);
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location (Loading
  // station)
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7.4, 2.8, 220 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(6.2, 2.3, .6);
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(7.4, 2.8, 0.2);
  StartPointTurn(-80 * (M_PI / 180.));
  Wait(50);
  success = StartDriveVisionBottom();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(25);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      6.6, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(4.6, 2., 0. * (M_PI / 180.), -1, true);
  Wait(25);
  GoTo(superstructure::HATCH_SHIP_FORWARDS);
  WaitUntilDriveComplete();
  ExitAutonomous();  // bye
}

void Rocket::LeftCargoRocket() {
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = 0;

  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running CARGO_RCOKET auto");

  GoTo(superstructure::CARGO_GROUND, superstructure::PREP_SCORE);
  StartDrivePath(5., .3, 0., 1, true, false, 2.0);
  WaitUntilDrivetrainNear(4.3, 1.3, .6);
  StartDriveVision(0.78);
  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      5.2, .3, (drive_status->estimated_heading() - init_gyro) + init_theta);
  GoTo(superstructure::CARGO_GROUND, superstructure::INTAKE_HATCH);
  StartDrivePath(1.6, 2.9, 0., -1, true);
  WaitUntilDriveComplete();
  StartPointTurn(-180 * (M_PI / 180.));
  Wait(100);

  bool success = StartDriveVision(0.69);
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7.4, 2.8, 220 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(6.2, 2.3, .6);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(7.4, 2.8, 0.2);
  StartPointTurn(-80 * (M_PI / 180.));
  Wait(50);
  ExitAutonomous();  // bye
  return;
}

void Rocket::RightRocket() {
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = M_PI;

  SetFieldPosition(1.8, -1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running NONE auto");

  StartDrivePath(7.4, -3.1, -210 * (M_PI / 180.), -1, true);
  GoTo(superstructure::CARGO_GROUND, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(7.4, -3.1, 0.2);
  StartPointTurn(60 * (M_PI / 180.));
  Wait(50);
  bool success = StartDriveVision();
  if (!success) {
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(25);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      6.4, -3.5, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7.1, -2.1, -150. * (M_PI / 180.), -1, true);

  WaitUntilDriveComplete();
  StartDrivePath(.4, -3.3, -M_PI, 1, true, true, 0, 0.5);
  Wait(150);
  GoTo(superstructure::HATCH_SHIP_FORWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(1.4, -3.4, 0.6);
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVision(0.65);
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  // Resetting field position again because we are in a known location (Loading
  // station)
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, -3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7.4, -2.8, -220 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(6.2, -2.3, .6);
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(7.4, -2.8, 0.2);
  StartPointTurn(80 * (M_PI / 180.));
  Wait(50);
  success = StartDriveVisionBottom();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(25);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      6.6, -3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(4.6, -2., 0. * (M_PI / 180.), -1, true);
  Wait(25);
  GoTo(superstructure::HATCH_SHIP_FORWARDS);
  WaitUntilDriveComplete();
  ExitAutonomous();  // bye
}

void Rocket::RightCargoRocket() {
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = 0;

  SetFieldPosition(1.8, -1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running CARGO_RCOKET auto");

  GoTo(superstructure::CARGO_GROUND, superstructure::PREP_SCORE);
  StartDrivePath(5., -.3, 0., 1, true, false, 2.0);
  WaitUntilDrivetrainNear(4.3, -1.3, .6);
  StartDriveVision(0.78);
  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      5.2, -.3, (drive_status->estimated_heading() - init_gyro) + init_theta);
  GoTo(superstructure::HATCH_SHIP_FORWARDS, superstructure::INTAKE_HATCH);
  StartDrivePath(1.6, -2.9, 0., -1, true);
  WaitUntilDriveComplete();
  StartPointTurn(180 * (M_PI / 180.));
  Wait(100);

  bool success = StartDriveVision(0.69);
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, -3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(7.4, -2.8, -220 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(6.2, -2.3, .6);
  GoTo(superstructure::CARGO_GROUND, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(7.4, -2.8, 0.2);
  StartPointTurn(80 * (M_PI / 180.));
  Wait(50);
  ExitAutonomous();  // bye
  return;
}

}  // namespace commands
}  // namespace c2019
