#include "c2019/commands/test_auto.h"

namespace c2019 {
namespace commands {

using muan::wpilib::DriverStationProto;

bool TestAuto::IsAutonomous() {
  DriverStationProto driver_station;
  AutoGoalProto auto_goal;
  if (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station status found.");
    return false;
  }

  if (!driver_station->is_sys_active()) {
    LOG(WARNING, "Tried to run command while disabled.");
    return false;
  }

  if (!auto_goal_reader_.ReadLastMessage(&auto_goal)) {
    LOG(WARNING, "No auto goal found.");
    return false;
  }

  if (auto_goal->run_command() && auto_goal->command() == TEST_AUTO) {
    return true;
  }

  return false;
}

void TestAuto::operator()() {
  EnterAutonomous();
  SetFieldPosition(0.0, 0.0, 0.0);
  LOG(INFO, "Running TEST auto");

  StartDrivePath(2.55, 1.2, 0, 1, false);
  Wait(50);
  ExitAutonomous();
}

}  // namespace commands
}  // namespace c2019
