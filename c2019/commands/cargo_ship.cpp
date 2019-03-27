#include "c2019/commands/cargo_ship.h"
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {
namespace commands {

using c2019::limelight::LimelightStatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

bool CargoShip::IsAutonomous() {
  DriverStationProto driver_station;
  if (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station status found.");
    return false;
  }

  if (!driver_station->is_sys_active()) {
    LOG(WARNING, "Tried to run command while disabled.");
    return false;
  }

  return driver_station->mode() == RobotMode::AUTONOMOUS;
}

void CargoShip::operator()() {
  EnterAutonomous();
  DrivetrainStatus drive_status;
  QueueManager<DrivetrainStatus>::Fetch()->ReadLastMessage(&drive_status);
  double init_heading = drive_status->estimated_heading();
  SetFieldPosition(1.8, 1.2, 0.0);
  LOG(INFO, "Running NONE auto");
  // Move to 1st level height & socre hatch L1 rocket
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);

  StartDrivePath(5.0, 4.0, 20 * (M_PI / 180.), 1, true, false);
  Wait(50);
  GoTo(superstructure::HATCH_ROCKET_FIRST, superstructure::PREP_SCORE);
  // Wann get reasonably close to rocket before starting vision, also enables
  // smooth transition to vision
  WaitUntilDrivetrainNear(1.8 + 1.4, 1.2 + 1.0, 0.6);
  // WaitForElevatorAndLL();
  ExitAutonomous();
  return;
}

}  // namespace commands
}  // namespace c2019
