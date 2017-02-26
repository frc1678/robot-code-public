#include "muan/wpilib/gyro/gyro_reader.h"
#include "muan/utils/history.h"
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
  aos::SetCurrentThreadName("Gyro");

  if (gyro_queue_ == nullptr) {
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
  while (calibration_state_ == GyroState::kUninitialized && !gyro_.InitializeGyro()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (gyro_queue_ != nullptr) {
      GyroMessageProto gyro_message;
      gyro_message->set_state(GyroState::kUninitialized);
      gyro_queue_->WriteMessage(gyro_message);
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
  muan::utils::History<double> raw_velocity{calib_cycles};

  if (calibration_state_ == GyroState::kInitialized) {
    calibration_state_ = GyroState::kCalibrating;
  }

  bool robot_disabled = true;
  while (calibration_state_ == GyroState::kCalibrating && robot_disabled) {
    raw_velocity.Update(AngleReading());

    // Send out a GyroMessage if the queue exists
    if (gyro_queue_ != nullptr) {
      GyroMessageProto gyro_message;
      gyro_message->set_state(GyroState::kCalibrating);
      gyro_message->set_calibration_time_left((calib_cycles - raw_velocity.num_samples()) *
                                              std::chrono::duration<double>(loop_time).count());
      gyro_queue_->WriteMessage(gyro_message);
    }

    phased_loop.SleepUntilNext();

    if (ds_queue_ != nullptr) {
      auto ds_message = ds_queue_->ReadLastMessage();
      if (ds_message) {
        robot_disabled = ds_message.value()->mode() == RobotMode::DISABLED;
      }
    }
  }

  double drift_sum = 0;
  for (auto i : raw_velocity) {
    drift_sum += i;
  }
  drift_rate_ = drift_sum / raw_velocity.num_samples();
}

void GyroReader::RunReader() {
  auto loop_time = std::chrono::milliseconds(5);
  aos::time::PhasedLoop phased_loop(loop_time);

  calibration_state_ = GyroState::kRunning;

  while (calibration_state_ == GyroState::kRunning) {
    double velocity = AngleReading() - drift_rate_;

    // Integrate the gyro readings - the drift rate is in radians per cycle
    angle_ += velocity * std::chrono::duration<double>(loop_time).count();

    // Reset if the should_reset_ flag is set, then clear it.
    if (should_reset_.exchange(false)) {
      angle_ = 0.0;
    }

    // Send out a GyroMessage if the queue exists
    if (gyro_queue_ != nullptr) {
      GyroMessageProto gyro_message;
      gyro_message->set_current_angle(angle_);
      gyro_message->set_current_angular_velocity(velocity);
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
