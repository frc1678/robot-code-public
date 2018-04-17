#ifndef C2018_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
#define C2018_AUTONOMOUS_AUTONOMOUS_RUNNER_H_

#include <string>

#include "c2018/autonomous/autonomous_base.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "c2018/autonomous/switch_only.h"
#include "c2018/autonomous/scale_only.h"
#include "c2018/autonomous/switch_and_scale.h"
#include "c2018/autonomous/backside_switch.h"
#include "c2018/autonomous/drive.h"
#include "c2018/autonomous/none.h"
#include "c2018/autonomous/sneak.h"

namespace c2018 {
namespace autonomous {
class AutonomousRunner {
 public:
  AutonomousRunner();
  void operator()();

 private:
  bool switch_only_ = false;
  bool scale_only_ = false;
  bool switch_and_scale_ = false;
  bool backside_switch_ = false;
  bool drive_ = false;
  bool none_ = false;
  bool sneak_ = false;

  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::webdash::AutoSelectionQueue::QueueReader auto_mode_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;

  std::string AutoMode();

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(5)};
};

}  // namespace autonomous
}  // namespace c2018

#endif  // C2018_AUTONOMOUS_AUTONOMOUS_RUNNER_H_
