#include "gyro_reader.h"
#include "third_party/aos/common/util/phased_loop.h"

// Gyro datasheet:
// http://www.analog.com/media/en/technical-documentation/data-sheets/ADXRS453.pdf

namespace muan {

namespace wpilib {

namespace gyro {

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

void GyroReader::Init() {
  // Try to initialize repeatedly every 100ms until it works.
  while (calibration_state_ == GyroState::kUninitialized &&
         !gyro_.InitializeGyro()) {
    aos::time::SleepFor(aos::time::Time::InMS(100));
    if (gyro_queue_ != nullptr) {
      StackGyroMessage gyro_message;
      gyro_message->set_state(GyroState::kUninitialized);
      gyro_queue_->WriteMessage(gyro_message);
    }
  }
  calibration_state_ = GyroState::kInitialized;
}

void GyroReader::RunCalibration() {
  aos::time::Time loop_time = aos::time::Time::InMS(5);
  aos::time::PhasedLoop phased_loop(loop_time);

  const aos::time::Time calib_time = aos::time::Time::InSeconds(15);

  size_t num_cycles;
  const size_t calib_cycles = calib_time / loop_time;

  double drift_sum = 0.0;

  if (calibration_state_ == GyroState::kInitialized) {
    calibration_state_ = GyroState::kCalibrating;
  }

  for (num_cycles = 0; num_cycles < calib_cycles &&
                       calibration_state_ == GyroState::kCalibrating;
       num_cycles++) {
    drift_sum += gyro_.ExtractAngle(gyro_.GetReading());

    // Send out a GyroMessage if the queue exists
    if (gyro_queue_ != nullptr) {
      StackGyroMessage gyro_message;
      gyro_message->set_state(GyroState::kCalibrating);
      gyro_message->set_calibration_time_left((calib_cycles - num_cycles) *
                                              loop_time.ToSeconds());
      gyro_queue_->WriteMessage(gyro_message);
    }

    phased_loop.SleepUntilNext();
  }

  drift_rate_ = drift_sum / num_cycles;
}

void GyroReader::RunReader() {
  aos::time::Time loop_time = aos::time::Time::InMS(5);
  aos::time::PhasedLoop phased_loop(loop_time);

  if (calibration_state_ == GyroState::kRunning) {
    calibration_state_ = GyroState::kCalibrating;
  }

  while (calibration_state_ == GyroState::kRunning) {
    double reading = gyro_.ExtractAngle(gyro_.GetReading());

    angle_ += gyro_.ExtractAngle(gyro_.GetReading()) * loop_time.ToSeconds() +
              drift_rate_;

    if (should_reset_.exchange(false)) {
      angle_ = 0.0;
    }

    // Send out a GyroMessage if the queue exists
    if (gyro_queue_ != nullptr) {
      StackGyroMessage gyro_message;
      gyro_message->set_current_angle(angle_);
      gyro_message->set_current_angular_velocity(reading);
      gyro_message->set_state(GyroState::kRunning);
      gyro_queue_->WriteMessage(gyro_message);
    }

    phased_loop.SleepUntilNext();
  }
}

}  // gyro

}  // wpilib

}  // muan
