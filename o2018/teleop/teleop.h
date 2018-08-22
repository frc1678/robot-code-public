#ifndef O2018_TELEOP_TELEOP_H_
#define O2018_TELEOP_TELEOP_H_

#include <atomic>
#include "WPILib.h"
#include "muan/queues/queue_manager.h"
#include "muan/utils/threading_utils.h"
#include "muan/wpilib/ds_sender.h"

namespace o2018 {
namespace teleop {

class TeleopBase {
 public:
  TeleopBase();

  void operator()();
  void Stop();

 private:
  std::atomic<bool> running_;

  void Update();

  // Message Sending functions go here
  // Buttons and joysticks go here

  muan::wpilib::DriverStationSender ds_sender_;
};

}  // namespace teleop
}  // namespace o2018

#endif  // O2018_TELEOP_TELEOP_H_
