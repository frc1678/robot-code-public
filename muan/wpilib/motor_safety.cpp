#include "motor_safety.h"

MotorSafety::MotorSafety(double current_threshold, double stall_time, double reset_time, double dt) : current_history_(dt) {
  current_threshold_ = current_threshold;
  stall_time_ = stall_time;
  reset_time_ = reset_time;
  dt_ = dt;

  time_above_ = time_below_ = 0;
  is_stalled_ = false;
}

double MotorSafety::Update(double voltage, double current) {

  current_history_.Update(current);

  // Take a moving average of the history array
  double sum = 0;
  std::array<double, kHistorySize> hist_arr = current_history_.get_hist_arr(); 
  for(size_t i = 0; i < kHistorySize; i++) {
      sum += hist_arr[i];
  }
  double moving_avg = sum / current_history_.num_samples();

  // Determine if the current is above the threshold or not
  if(moving_avg >= current_threshold_) {
    time_above_ += dt_;
    time_below_ = 0;
  } else {
    time_above_ = 0;
    time_below_ += dt_;
  }

  // Determine if the motor has been stalling
  if(time_above_ >= stall_time_) {
    is_stalled_ = true;
  } else if (time_below_ > reset_time_) {
    is_stalled_ = false;
  }

  return is_stalled_ ? 0 : voltage;
}

void MotorSafety::Reset() {
  is_stalled_ = false;
  time_above_ = 0;
  time_below_ = 0;
}

bool MotorSafety::is_stalled() { return is_stalled_; }
