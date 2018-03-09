#ifndef C2018_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
#define C2018_AUTONOMOUS_AUTONOMOUS_RUNNER_H_

#include "c2018/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace c2018 {
namespace autonomous {
class AutonomousRunner {
 public:
  AutonomousRunner();
  void operator()();

 private:
  bool switch_only_ = true;  // TODO(Livy) use webdash sender stuff instead
  bool scale_only_ = false;
  bool switch_and_scale_ = false;

  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(5)};
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
