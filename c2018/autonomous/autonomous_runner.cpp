#include "c2018/autonomous/autonomous_runner.h"

namespace c2018 {
namespace autonomous {

using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;

AutonomousRunner::AutonomousRunner()
    : driver_station_reader_(
          QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      game_specific_string_reader_(
          QueueManager<muan::wpilib::GameSpecificStringProto>::Fetch()
              ->MakeReader()) {}

void AutonomousRunner::operator()() {
  aos::SetCurrentThreadRealtimePriority(10);
  muan::utils::SetCurrentThreadName("Autonomous");

  muan::wpilib::DriverStationProto driver_station;
  muan::wpilib::GameSpecificStringProto game_specific_string;

  while (!driver_station_reader_.ReadLastMessage(&driver_station)) {
    LOG(WARNING, "No driver station message!");
    loop_.SleepUntilNext();
  }

  while (driver_station_reader_.ReadLastMessage(&driver_station),
         driver_station->mode() != RobotMode::AUTONOMOUS) {
    loop_.SleepUntilNext();
  }

  while (!game_specific_string_reader_.ReadLastMessage(&game_specific_string)) {
    LOG(ERROR, "Waiting on auto because there's no game specific message yet!");
    loop_.SleepUntilNext();
  }

  // SwitchOnly().LeftSwitch();
  // return;

  // Start of autonomous. Grab the game specific string.
  auto left_right_codes = game_specific_string->code();
  LOG(INFO, "Starting autonomous with layout %s", left_right_codes.c_str());
  if (switch_only_) {
    if (left_right_codes[0] == 'L') {
      // Left switch only
    } else if (left_right_codes[0] == 'R') {
      // Right switch only
    }
  } else if (scale_only_) {
    if (left_right_codes[1] == 'L') {
      // Left scale only
    } else if (left_right_codes[1] == 'R') {
      // Right scale only
    }
  } else if (switch_and_scale_) {
    if (left_right_codes[0] == 'L') {
      if (left_right_codes[1] == 'L') {
        // LEFT LEFT
      } else if (left_right_codes[1] == 'R') {
        // LEFT RIGHT
      }
    } else if (left_right_codes[0] == 'R') {
      if (left_right_codes[1] == 'L') {
        // RIGHT LEFT
      } else if (left_right_codes[1] == 'R') {
        // RIGHT RIGHT
      }
    }
  } else {
    LOG(WARNING, "No auto mode found!");
  }
  LOG(INFO, "Finished with auto!");
}

}  // namespace autonomous
}  // namespace c2018
