#ifndef MUAN_UTILS_MONITOR_H_
#define MUAN_UTILS_MONITOR_H_

#include "muan/utils/history.h"

namespace muan {

namespace utils {

class Monitor {
 public:
  Monitor(double threshold = 120., double delay_time = 2., double reset_time = 2., double dt = 0.005, bool check_above = true, double standing_voltage = 0.);
  // threshold is the value you must either surpass or go below
  // delay_time is the amount of time before it considers itself above or below te threshold
  // reset_time is the time it takes before it resets to reading live voltages
  // dt is the rate at which the Monitor runs
  // check_above determines if you want to check if the value is above or below the threshold
  // standing_voltage is how much voltage it returns if it's triggered the Monitor

  double Update(double voltage, double value);
  void Reset();
  bool is_at_thresh();

 private:
  double threshold_, delay_time_, reset_time_, dt_, check_above_, standing_voltage_;

  double time_above_, time_below_;
  bool is_at_thresh_;

  static constexpr uint32_t kHistorySize = 20;
  muan::utils::History<double, kHistorySize> current_history_;
};

}  // utils 

}  // muan

#endif  // MUAN_UTILS_MONITOR_H_
