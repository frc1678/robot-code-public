#ifndef MUAN_WPILIB_MOTOR_SAFETY_H_
#define MUAN_WPILIB_MOTOR_SAFETY_H_

#include "muan/utils/history.h"

class MotorSafety {
 public:
  MotorSafety(double current_threshold = 120., double stall_time = 2., double reset_time = 2.,
              double dt = 0.005);

  // Returns voltage
  double Update(double voltage, double current);
  void Reset();
  bool is_stalled();

 private:
  double current_threshold_, stall_time_, reset_time_, dt_;

  double time_above_, time_below_;
  bool is_stalled_;

  static constexpr uint32_t kHistorySize = 20;
  muan::History<double, kHistorySize> current_history_;
};

#endif  // MUAN_WPILIB_MOTOR_SAFETY_H_
