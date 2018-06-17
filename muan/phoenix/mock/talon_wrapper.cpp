#include "muan/phoenix/mock/talon_wrapper.h"
#include "muan/units/units.h"

namespace muan {
namespace phoenix {

TalonWrapper::TalonWrapper(int id, Config config)
    : id_(id), conversion_factor_(config.conversion_factor) {}

void TalonWrapper::SetOpenloopGoal(double setpoint) {  // Voltage
  open_loop_voltage_ = setpoint;
}

void TalonWrapper::SetPositionGoal(double setpoint,
                                   double /* setpoint_ff */) {  // Position, Voltage
  prev_position_ = position_;
  position_ = setpoint;
}

void TalonWrapper::SetVelocityGoal(double setpoint,
                                   double /* setpoint_ff */) {  // Velocity, Voltage
  prev_position_ = position_;
  position_ += setpoint * 10 * ms;
}

void TalonWrapper::SetGains(Gains /* gains */, int /* slot */) {}

void TalonWrapper::SelectGains(int /* slot */) {}

}  // namespace phoenix
}  // namespace muan
