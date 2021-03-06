#ifndef C2019_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
#define C2019_AUTONOMOUS_AUTONOMOUS_RUNNER_H_

#include <string>

#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace c2019 {
namespace autonomous {

class AutonomousRunner {
 public:
  AutonomousRunner();
  void operator()();

 private:
  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::webdash::AutoQueue::QueueReader auto_mode_reader_;

  std::string AutoMode();

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
};

}  // namespace autonomous
}  // namespace c2019

#endif  // C2019_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
