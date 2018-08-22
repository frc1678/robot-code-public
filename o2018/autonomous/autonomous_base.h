#ifndef O2018_AUTONOMOUS_AUTONOMOUS_BASE_H_
#define O2018_AUTONOMOUS_AUTONOMOUS_BASE_H_

#include <string>

#include "gtest/gtest.h"
#include "muan/webdash/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/util/phased_loop.h"

namespace o2018 {
namespace autonomous {

class AutonomousBase {
 public:
  AutonomousBase();

 protected:
  bool IsAutonomous();

  void Wait(uint32_t num_cycles);

  // Set the robot-space (robot poweron position) transformation. The parameters
  // are the position of the robot (right now) in field coordinates (F).
  // TODO(jishnusen) Actually implement this once drivertrain code is merged
  /* void SetFieldPosition(double x, double y, double theta); */

  muan::wpilib::DriverStationQueue::QueueReader driver_station_reader_;
  muan::wpilib::GameSpecificStringQueue::QueueReader
      game_specific_string_reader_;

  aos::time::PhasedLoop loop_{std::chrono::milliseconds(10)};
};

}  // namespace autonomous
}  // namespace o2018

#endif  // O2018_AUTONOMOUS_AUTONOMOUS_BASE_H_
