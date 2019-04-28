#include "c2019/commands/rocket.h"
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {
using c2019::limelight::LimelightStatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

void Rocket::RightBackRocket() {
  max_lin_ = 3.0;
  max_acc_ = 3.5;
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = M_PI;

  SetFieldPosition(1.8, -1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running RIGHT BACK ROCKET auto");

  StartDrivePath(7.7, -3.1, -210 * (M_PI / 180.), -1, true);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  GoTo(superstructure::REZERO);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  StartPointTurn(60 * (M_PI / 180.));
  WaitUntilDriveComplete();
  bool success = StartDriveVision();
  if (!success) {
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      6.4, -3.5, (drive_status->estimated_heading() - init_gyro) + init_theta);
  StartDrivePath(7.1, -2.1, -160. * (M_PI / 180.), -1, true);
  /* final_vel_ = 1.5; */

  WaitUntilDriveComplete();
  StartDrivePath(.4, -3.1, -170 * (M_PI / 180.), 1, true, true, 0, 0.5);
  /* final_vel_ = 0.0; */
  /* Wait(150); */

  WaitUntilDrivetrainNear(1.4, -3.4, 0.6);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::INTAKE_HATCH);
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVision();
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

  StartDrivePath(7.6, -2.8, -220 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(6.2, -2.2, .6);
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  StartPointTurn(90 * (M_PI / 180.));
  WaitUntilDriveComplete();
  success = StartDriveVisionBottom();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(50);
  SetFieldPosition(
      6.4, -3.5, (drive_status->estimated_heading() - init_gyro) + init_theta);
  StartDrivePath(7.1, -2.1, -170. * (M_PI / 180.), -1, true);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::INTAKE_HATCH);
  WaitUntilDriveComplete();
  ExitAutonomous();
}

void Rocket::LeftBackRocket() {
  max_lin_ = 3.0;
  max_acc_ = 3.5;
  EnterAutonomous();
  // Set field position to right side of L1 HAB
  double init_theta = M_PI;

  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running LEFT BACK ROCKET auto");

  StartDrivePath(7.7, 3.1, 210 * (M_PI / 180.), -1, true);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  GoTo(superstructure::REZERO);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  StartPointTurn(-60 * (M_PI / 180.));
  WaitUntilDriveComplete();
  bool success = StartDriveVision();
  if (!success) {
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      6.4, 3.5, (drive_status->estimated_heading() - init_gyro) + init_theta);
  StartDrivePath(7.1, 2.1, 160. * (M_PI / 180.), -1, true);
  /* final_vel_ = 1.5; */

  WaitUntilDriveComplete();
  StartDrivePath(.4, 3.1, 170 * (M_PI / 180.), 1, true, true, 0, 0.5);
  /* final_vel_ = 0.0; */
  /* Wait(150); */

  WaitUntilDrivetrainNear(1.4, 3.4, 0.6);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::INTAKE_HATCH);
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVision();
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

  StartDrivePath(7.6, 2.8, 220 * (M_PI / 180.), -1, true);
  WaitUntilDrivetrainNear(6.2, 2.2, .6);
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  StartPointTurn(-90 * (M_PI / 180.));
  WaitUntilDriveComplete();
  success = StartDriveVisionBottom();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(50);
  SetFieldPosition(
      6.4, 3.5, (drive_status->estimated_heading() - init_gyro) + init_theta);
  StartDrivePath(7.1, 2.1, 170. * (M_PI / 180.), -1, true);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::INTAKE_HATCH);
  WaitUntilDriveComplete();
  ExitAutonomous();
}

void Rocket::RightDoubleRocket() {
  EnterAutonomous();
  max_lin_ = 3.0;
  max_acc_ = 3.0;

  // Set field position to right side of L1 HAB
  double init_theta = M_PI;

  SetFieldPosition(1.8, -1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running RIGHT DOUBLE ROCKET auto");

  StartDrivePath(7.7, -3.1, -210 * (M_PI / 180.), -1, true);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  GoTo(superstructure::REZERO);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(5.8, -2.0, .5);
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  Wait(15);
  StartPointTurn(60 * (M_PI / 180.));
  WaitUntilDriveComplete();

  bool success = StartDriveVisionBottom();
  if (!success) {
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      6.4, -3.5, (drive_status->estimated_heading() - init_gyro) + init_theta);
  final_vel_ = 2.0;
  StartDrivePath(5.5, -2.1, 0.0, -1, true, true);
  Wait(15);
  GoTo(superstructure::GROUND, superstructure::INTAKE_HATCH);
  final_vel_ = 0.0;
  WaitUntilDriveComplete();
  StartDrivePath(.4, -3.6, 0 * (M_PI / 180.), -1, true, true);

  /* WaitUntilDrivetrainNear(3.3, 3.3, .6); */
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(1.9, -3.5, 0.6);
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVisionBackwards();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  max_lin_ = 2.7;
  max_acc_ = 2.7;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, -3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(4.9, -3.3, -34 * (M_PI / 180.), 1, true);
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(4.9, -3.3, 0.6);
  success = StartDriveVisionBottom();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(50);

  max_lin_ = 3.5;
  max_acc_ = 4.0;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      4.9, -3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  /* final_vel_ = 1.0; */
  StartDrivePath(4.0, -2.7, 10. * (M_PI / 180.), -1, true);
  Wait(80);
  GoTo(superstructure::CARGO_AUTO, superstructure::INTAKE_CARGO);
  WaitUntilDriveComplete();
  final_vel_ = 0.0;
  StartDrivePath(6.35, -1.3, 90 * (M_PI / 180.), 1, true);
  Wait(10);
  GoTo(superstructure::CARGO_AUTO, superstructure::INTAKE_CARGO);
  WaitUntilDriveComplete();
  ExitAutonomous();  // bye
}

void Rocket::LeftDoubleRocket() {
  EnterAutonomous();
  max_lin_ = 3.0;
  max_acc_ = 3.0;

  // Set field position to right side of L1 HAB
  double init_theta = M_PI;

  SetFieldPosition(1.8, 1.1, init_theta);
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_gyro = drive_status->estimated_heading();
  LOG(INFO, "Running LEFT DOUBLE ROCKET auto");

  StartDrivePath(7.7, 3.1, 210 * (M_PI / 180.), -1, true);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  GoTo(superstructure::REZERO);
  GoTo(superstructure::GROUND, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(5.8, 2.0, .5);
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  WaitUntilDriveComplete();
  Wait(15);
  StartPointTurn(-60 * (M_PI / 180.));
  WaitUntilDriveComplete();

  bool success = StartDriveVisionBottom();
  if (!success) {
    ExitAutonomous();
    return;
  }
  ScoreHatch(1);
  Wait(50);

  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      6.4, 3.5, (drive_status->estimated_heading() - init_gyro) + init_theta);
  final_vel_ = 2.0;
  StartDrivePath(5.5, 2.1, 0.0, -1, true, true);
  Wait(15);
  GoTo(superstructure::GROUND, superstructure::INTAKE_HATCH);
  final_vel_ = 0.0;
  WaitUntilDriveComplete();
  StartDrivePath(.4, 3.6, 0 * (M_PI / 180.), -1, true, true);

  /* WaitUntilDrivetrainNear(3.3, 3.3, .6); */
  GoTo(superstructure::HATCH_ROCKET_BACKWARDS, superstructure::INTAKE_HATCH);

  WaitUntilDrivetrainNear(1.9, 3.5, 0.6);
  // Activate vision once dt is reasonably near loading station
  success = StartDriveVisionBackwards();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  max_lin_ = 2.7;
  max_acc_ = 2.7;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      0.4, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  StartDrivePath(4.9, 3.3, 34 * (M_PI / 180.), 1, true);
  GoTo(superstructure::HATCH_ROCKET_SECOND, superstructure::PREP_SCORE);
  WaitUntilDrivetrainNear(4.9, 3.3, 0.6);
  success = StartDriveVisionBottom();
  if (!success) {
    LOG(WARNING, "Vision didn't work");
    ExitAutonomous();
    return;
  }

  ScoreHatch(1);
  Wait(50);

  max_lin_ = 3.5;
  max_acc_ = 4.0;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  SetFieldPosition(
      4.9, 3.4, (drive_status->estimated_heading() - init_gyro) + init_theta);

  /* final_vel_ = 1.0; */
  StartDrivePath(4.0, 2.7, -10. * (M_PI / 180.), -1, true);
  Wait(80);
  GoTo(superstructure::CARGO_AUTO, superstructure::INTAKE_CARGO);
  WaitUntilDriveComplete();
  final_vel_ = 0.0;
  StartDrivePath(6.45, 1.3, -90 * (M_PI / 180.), 1, true);
  Wait(10);
  GoTo(superstructure::CARGO_AUTO, superstructure::INTAKE_CARGO);
  WaitUntilDriveComplete();
  ExitAutonomous();  // bye
}

}  // namespace commands
}  // namespace c2019
