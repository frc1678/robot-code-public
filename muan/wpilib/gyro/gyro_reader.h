#ifndef MUAN_WPILIB_GYRO_GYRO_READER_H_
#define MUAN_WPILIB_GYRO_GYRO_READER_H_

#include <WPILib.h>
#include "muan/wpilib/gyro/gyro_interface.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/units/units.h"
#include "muan/wpilib/gyro/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/aos/common/time.h"

namespace muan {

namespace wpilib {

namespace gyro {

using namespace muan::units;  // NOLINT

class GyroReader {
 public:
  // If invert is set to true, clockwise on the robot is positive. Useful for
  // when mechanical decides that the RoboRIO should be upside down for some reason.
  explicit GyroReader(GyroQueue* queue, DriverStationQueue* ds_queue, int calib_time, bool invert = false);

  // Run forever. This should be passed as an argument to the constructor of
  // std::thread.
  void operator()();

  void Reset();
  void Recalibrate();
  void Quit();

 protected:
  void Init();
  void RunCalibration();
  void RunReader();

  bool is_calibrated();

  // Read from the gyro, accounting for whether or not the gyro is inverted.
  double AngleReading();

  GyroInterface gyro_;

  GyroQueue* gyro_queue_;

  muan::wpilib::DriverStationQueue* ds_queue_;

  // How long should we calibrate for?
  const std::chrono::seconds calib_time_;

  // Is the RIO upside down?
  bool should_invert_;

  // The rate at which the gyro will drift, in radians per tick (at 200hz)
  double drift_rate_ = 0;

  // The integrated angle
  double angle_{0.0};

  std::atomic<bool> should_reset_{false};

  std::atomic<GyroState> calibration_state_{GyroState::kUninitialized};

  // Based on logs, this gives plenty of room for bad luck, as actual values
  // were less than 0.015, but being pushed will exceed this value.
  const double calibration_velocity_limit_ = 0.05;
};

}  // namespace gyro

}  // namespace wpilib

}  // namespace muan

#endif  // MUAN_WPILIB_GYRO_GYRO_READER_H_
