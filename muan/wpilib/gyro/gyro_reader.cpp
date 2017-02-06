#include "muan/wpilib/gyro/gyro_reader.h"
#include "third_party/aos/common/util/phased_loop.h"

// Gyro datasheet:
// http://www.analog.com/media/en/technical-documentation/data-sheets/ADXRS453.pdf

namespace muan {

namespace wpilib {

namespace gyro {

GyroReader::GyroReader(GyroQueue* queue, bool invert, int calib_time) :
    gyro_queue_{queue},
    should_invert_{invert},
    calib_time_{std::chrono::seconds(calib_time)} {}

void GyroReader::Reset() { should_reset_ = true; }

void GyroReader::operator()() {
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
  const size_t wait_cycles = wait_time / loop_time;
  for (size_t num_cycles = 0; num_cycles < wait_cycles; num_cycles++) {
    gyro_.GetReading();
  }

  // Setup for averaging over calibration period
  size_t num_cycles;
  const size_t calib_cycles = calib_time_ / loop_time;
  double drift_sum = 0.0;

  if (calibration_state_ == GyroState::kInitialized) {
    calibration_state_ = GyroState::kCalibrating;
  }

  for (num_cycles = 0; num_cycles < calib_cycles && calibration_state_ == GyroState::kCalibrating;
       num_cycles++) {
    drift_sum += AngleReading();

    // Send out a GyroMessage if the queue exists
    if (gyro_queue_ != nullptr) {
      GyroMessageProto gyro_message;
      gyro_message->set_state(GyroState::kCalibrating);
      gyro_message->set_calibration_time_left((calib_cycles - num_cycles) *
                                              std::chrono::duration<double>(loop_time).count());
      gyro_queue_->WriteMessage(gyro_message);
    }

    phased_loop.SleepUntilNext();
  }

  drift_rate_ = drift_sum / num_cycles;
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

void GyroReader::Quit() { calibration_state_ = GyroState::kKilled; }

void GyroReader::Recalibrate() { calibration_state_ = GyroState::kInitialized; }

}  // namespace gyro

}  // namespace wpilib

}  // namespace muan
