#include <sstream>
#include <string>
#include <vector>

#include "o2018/autonomous/autonomous_runner.h"
#include "o2018/autonomous/test_auto.h"
#include "o2018/autonomous/none.h"
#include "o2018/autonomous/drive_straight.h"

namespace o2018 {
namespace autonomous {

using muan::queues::QueueManager;
using muan::webdash::AutoProto;
using muan::webdash::WebDashQueueWrapper;
using muan::wpilib::DriverStationProto;

AutonomousRunner::AutonomousRunner()
    : driver_station_reader_(
          QueueManager<DriverStationProto>::Fetch()->MakeReader()),
      auto_mode_reader_(WebDashQueueWrapper::GetInstance()
                            .auto_queue()
                            .MakeReader()),
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

  auto left_right_codes = game_specific_string->code();

  if (AutoMode() == "TEST_AUTO") {
    TestAuto test_auto;
    if (left_right_codes[0] == 'L') {
      test_auto.LeftSwitch();
    } else if (left_right_codes[0] == 'R') {
      test_auto.RightSwitch();
    }
  } else if (AutoMode() == "DRIVE_STRAIGHT") {
    DriveStraight drive_auto;
    drive_auto.Drive();
  } else {
    None none_auto;
    none_auto.NoneAuto();
  }
}

std::string AutonomousRunner::AutoMode() {
  AutoProto auto_mode;
  std::string final_auto_mode;
  muan::wpilib::GameSpecificStringProto game_specific_string;
  game_specific_string_reader_.ReadLastMessage(&game_specific_string);
  if (auto_mode_reader_.ReadLastMessage(&auto_mode)) {
    std::vector<std::string> autos;
    std::string each_auto;
    std::istringstream auto_stream(auto_mode->auto_modes());
    while (std::getline(auto_stream, each_auto, ';')) {
      autos.push_back(each_auto);
    }
    for (std::string &autonomous_mode : autos) {
      if (game_specific_string->code().substr(0, 2) ==
          autonomous_mode.substr(0, 2)) {
        final_auto_mode = autonomous_mode.substr(3, autonomous_mode.size() - 3);
      }
    }
    return final_auto_mode;
  } else {
    return "DRIVE";
  }
}

}  // namespace autonomous
}  // namespace o2018
