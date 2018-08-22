#ifndef O2018_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
#define O2018_AUTONOMOUS_AUTONOMOUS_RUNNER_H_

#include <string>

#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "o2018/autonomous/autonomous_base.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace o2018 {
namespace autonomous {

class AutonomousRunner {
 public:
  AutonomousRunner();
  void operator()();

 private:
  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::webdash::AutoSelectionQueue::QueueReader auto_mode_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;

  std::string AutoMode();

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
};

}  // namespace autonomous
}  // namespace o2018

#endif  // O2018_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
