#ifndef C2018_SUBSYSTEMS_CLIMBER_WINCH_WINCH_H_
#define C2018_SUBSYSTEMS_CLIMBER_WINCH_WINCH_H_

#include <cmath>

namespace c2018 {

namespace climber {

namespace winch {

constexpr double kWinchRadius = 3;     // WRONG
constexpr double kAmountToClimb = 14;  // Inches
constexpr double kRunningVoltage = 6.0;

class Winch {
 public:
  Winch();

  double Update(double encoder_i, bool should_climb, bool outputs_enabled);

  bool is_reset() { return reset_; }
  bool has_climbed() { return has_climbed_; }

 private:
  double rope_climbed_;
  bool reset_ = true;
  double first_enc_pos_;
  bool has_climbed_;
};

}  // namespace winch

}  // namespace climber

}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_CLIMBER_WINCH_WINCH_H_
