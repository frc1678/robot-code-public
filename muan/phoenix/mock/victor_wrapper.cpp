#include "muan/phoenix/mock/victor_wrapper.h"
#include "muan/units/units.h"

namespace muan {
namespace phoenix {

VictorWrapper::VictorWrapper(int id, Config config)
    : id_(id), conversion_factor_(config.conversion_factor) {}

void VictorWrapper::SetOpenloopGoal(double setpoint) {  // Voltage
  open_loop_voltage_ = setpoint;
}

void VictorWrapper::SetPositionGoal(double setpoint,
                                    double /* setpoint_ff */) {  // Position, Voltage
  prev_position_ = position_;
  position_ = setpoint;
}

void VictorWrapper::SetVelocityGoal(double setpoint,
                                    double /* setpoint_ff */) {  // Velocity, Voltage
  prev_position_ = position_;
  position_ += setpoint * 10 * ms;
}

void VictorWrapper::SetGains(Gains /* gains */, int /* slot */) {}

void VictorWrapper::SelectGains(int /* slot */) {}

}  // namespace phoenix
}  // namespace muan
