#ifndef MUAN_UTILS_MONITOR_H_
#define MUAN_UTILS_MONITOR_H_

#include <cinttypes>
#include "muan/utils/history.h"

namespace muan {
namespace utils {

//  This Monitor class is intended to watch a value and return a voltage that
//  changes based off of whether or
//  not the value is above a threshold value. It's most commonly used for motor
//  safety and current regulation.
class Monitor {
 public:
  Monitor(double threshold = 120., double delay_time = 2.,
          double reset_time = 2., double dt = 0.005, bool check_above = true,
          double standing_voltage = 0., size_t size = 20);
  // threshold is the value you must either surpass or go below
  // delay_time is the amount of time before it considers itself above or below
  // te threshold
  // reset_time is the time it takes before it resets to reading live voltages
  // dt is the rate at which the Monitor runs
  // check_above determines if you want to check if the value is above or below
  // the threshold
  // standing_voltage is how much voltage it returns if it's triggered the
  // Monitor
  // size is the amount of time the moving average keeps track of (in seconds)

  double Update(double voltage, double value);
  void Reset();
  bool is_at_thresh();

 private:
  double threshold_, delay_time_, reset_time_, dt_, check_above_,
      standing_voltage_;

  double time_above_, time_below_;
  bool is_at_thresh_;

  size_t size_;
  muan::utils::History<double> current_history_;
};

}  // namespace utils
}  // namespace muan

#endif  // MUAN_UTILS_MONITOR_H_
