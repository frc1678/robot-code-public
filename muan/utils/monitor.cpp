#include "muan/utils/monitor.h"
#include "muan/logging/logger.h"

namespace muan {

namespace utils {

Monitor::Monitor(double threshold, double delay_time, double reset_time, double dt, bool check_above,
                 double standing_voltage, size_t size)
    : current_history_(size) {
  threshold_ = threshold;
  delay_time_ = delay_time;
  reset_time_ = reset_time;
  dt_ = dt;
  check_above_ = check_above;
  standing_voltage_ = standing_voltage;

  time_above_ = time_below_ = 0;
  is_at_thresh_ = false;
  size_ = size;
}

double Monitor::Update(double voltage, double value) {
  current_history_.Update(value);

  // Take a moving average of the history array
  double sum = 0;

  for (auto i : current_history_) {
    sum += i;
  }
  double moving_avg = sum / current_history_.num_samples();

  // Determine if the value is above/below the threshold or not
  if ((moving_avg >= threshold_) == check_above_) {
    time_above_ += dt_;
    time_below_ = 0;
  } else {
    time_above_ = 0;
    time_below_ += dt_;
  }

  // Determine if the time threshold has been met
  if (time_above_ >= delay_time_) {
    is_at_thresh_ = true;
  } else if (time_below_ > reset_time_) {
    is_at_thresh_ = false;
  }

  return is_at_thresh_ ? standing_voltage_ : voltage;
}

void Monitor::Reset() {
  is_at_thresh_ = false;
  time_above_ = 0;
  time_below_ = 0;
}

bool Monitor::is_at_thresh() { return is_at_thresh_; }

}  // namespace utils

}  // namespace muan
