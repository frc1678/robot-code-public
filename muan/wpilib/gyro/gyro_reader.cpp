#include "muan/logging/logger.h"
#include "muan/wpilib/gyro/gyro_reader.h"
#include "muan/utils/history.h"
#include "muan/utils/threading_utils.h"
#include "third_party/aos/common/util/phased_loop.h"
#include "third_party/aos/linux_code/init.h"

// Gyro datasheet:
// http://www.analog.com/media/en/technical-documentation/data-sheets/ADXRS453.pdf

namespace muan {

namespace wpilib {

namespace gyro {

GyroReader::GyroReader(GyroQueue* queue, DriverStationQueue* ds_queue, int calib_time, bool invert)
    : gyro_queue_{queue},
      ds_queue_{ds_queue},
      calib_time_{std::chrono::seconds(calib_time)},
      should_invert_{invert} {}

void GyroReader::operator()() {
  aos::SetCurrentThreadRealtimePriority(30);
  muan::utils::SetCurrentThreadName("Gyro");

  if (gyro_queue_ == nullptr) {
    LOG_P("No queue provided to gyro reader");
    aos::Die(
        "Please supply a queue to the gyro reader - otherwise there's no "
        "reason to run it!");
  }

  Init();
  RunCalibration();
  RunReader();
}

double GyroReader::AngleReading() {
  double angle = gyro_.ExtractAngle(gyro_.GetReading());
  if (should_invert_) {
    angle = -angle;
  }
  return angle;
}

void GyroReader::Init() {
  // Try to initialize repeatedly every 100ms until it works.
  LOG_P("Initializing!");
  aos::time::PhasedLoop phased_loop(std::chrono::milliseconds(100));
  while (calibration_state_ == GyroState::kUninitialized && !gyro_.InitializeGyro()) {
    LOG_P("Init failed");
    phased_loop.SleepUntilNext();
    if (gyro_queue_ != nullptr) {
      GyroMessageProto gyro_message;
      gyro_message->set_state(GyroState::kUninitialized);
      gyro_queue_->WriteMessage(gyro_message);
      LOG_P("Uninitialized");
    }
  }
  calibration_state_ = GyroState::kInitialized;
}

void GyroReader::RunCalibration() {
  auto loop_time = std::chrono::milliseconds(5);
  aos::time::PhasedLoop phased_loop(loop_time);

  const auto wait_time = std::chrono::seconds(5);
  const int wait_cycles = wait_time / loop_time;
  for (int num_cycles = 0; num_cycles < wait_cycles; num_cycles++) {
    gyro_.GetReading();
  }

  // Setup for averaging over calibration period
  const int calib_cycles = calib_time_ / loop_time;
  muan::utils::History<double> velocity_history{calib_cycles};

  if (calibration_state_ == GyroState::kInitialized) {
    calibration_state_ = GyroState::kCalibrating;
    LOG_P("Calibrating");
  }

  bool robot_disabled = true;
  if (ds_queue_ != nullptr) {
    auto ds_message = ds_queue_->ReadLastMessage();
    if (ds_message) {
      robot_disabled = ds_message.value()->mode() == RobotMode::DISABLED;
    }
  }

  int highest_sample_count = 0;
  while (calibration_state_ == GyroState::kCalibrating && robot_disabled) {
    double raw_velocity = AngleReading();

    // Reset calibration if robot is moved
    if (std::abs(raw_velocity) < kCalibrationVelocityLimit) {
      velocity_history.Update(raw_velocity);
    } else {
      velocity_history.reset();
      LOG_P("Velocity limit exceeded, Calibration Reset");
    }

    // Send out a GyroMessage if the queue exists
    if (gyro_queue_ != nullptr) {
      GyroMessageProto gyro_message;
      gyro_message->set_state(GyroState::kCalibrating);
      gyro_message->set_calibration_time_left((calib_cycles - velocity_history.num_samples()) *
                                              std::chrono::duration<double>(loop_time).count());
      gyro_message->set_raw_angular_velocity(raw_velocity);
      gyro_queue_->WriteMessage(gyro_message);
    }

    // Use the most recent complete calibration. If no calibration was
    // fully complete, use the one with the most samples.
    if (velocity_history.num_samples() >= highest_sample_count) {
      double drift_sum = 0;
      for (auto i : velocity_history) {
        drift_sum += i;
      }
      drift_rate_ = drift_sum / velocity_history.num_samples();
      highest_sample_count = velocity_history.num_samples();
    }

    phased_loop.SleepUntilNext();

    if (ds_queue_ != nullptr) {
      auto ds_message = ds_queue_->ReadLastMessage();
      if (ds_message) {
        robot_disabled = ds_message.value()->mode() == RobotMode::DISABLED;
      } else {
          LOG_P("The driverstation queue is null");
      }
    }
  }
}

void GyroReader::RunReader() {
  auto loop_time = std::chrono::milliseconds(5);
  aos::time::PhasedLoop phased_loop(loop_time);

  calibration_state_ = GyroState::kRunning;

  while (calibration_state_ == GyroState::kRunning) {
    double raw_velocity = AngleReading();
    double velocity = raw_velocity - drift_rate_;

    // Integrate the gyro readings - the drift rate is in radians per cycle
    angle_ += velocity * std::chrono::duration<double>(loop_time).count();

    // Reset if the should_reset_ flag is set, then clear it.
    if (should_reset_.exchange(false)) {
      angle_ = 0.0;
      LOG_P("reset_ flag sent, Resetting");
    }

    // Send out a GyroMessage if the queue exists
    if (gyro_queue_ != nullptr) {
      GyroMessageProto gyro_message;
      gyro_message->set_current_angle(angle_);
      gyro_message->set_current_angular_velocity(velocity);
      gyro_message->set_raw_angular_velocity(raw_velocity);
      gyro_message->set_state(GyroState::kRunning);
      gyro_queue_->WriteMessage(gyro_message);
    }

    phased_loop.SleepUntilNext();
  }
}

void GyroReader::Reset() { should_reset_ = true; }

void GyroReader::Quit() { calibration_state_ = GyroState::kKilled; }

void GyroReader::Recalibrate() { calibration_state_ = GyroState::kInitialized; }

}  // namespace gyro

}  // namespace wpilib

}  // namespace muan
