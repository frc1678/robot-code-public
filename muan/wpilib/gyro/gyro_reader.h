#ifndef DRIVECONTROLLER_H_
#define DRIVECONTROLLER_H_

#include "gyro_interface.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/units/units.h"
#include "muan/wpilib/gyro/gyro.pb.h"
#include "third_party/aos/common/time.h"
#include <WPILib.h>

namespace muan {

namespace wpilib {

namespace gyro {

using namespace muan::units;

class GyroReader {
 public:
  GyroReader() = default;

  // Run forever. This should be passed as an argument to the constructor of
  // std::thread.
  void operator()();

  void Reset();
  void Recalibrate();
  void Quit();

  using StackGyroMessage = muan::proto::StackProto<GyroMessage, 128>;
  using GyroQueue = muan::queues::MessageQueue<StackGyroMessage, 200>;

 protected:
  void Init();
  void RunCalibration();
  void RunReader();

  bool is_calibrated();

  GyroInterface gyro_;

  GyroQueue* gyro_queue_;

  // The rate at which the gyro will drift, in radians per tick (at 200hz)
  double drift_rate_ = 0;

  // The integrated angle
  double angle_{0.0};

  std::atomic<bool> should_reset_{false};

  std::atomic<GyroState> calibration_state_{GyroState::kUninitialized};
};

}  // gyro

}  // wpilib

}  // muan

#endif
