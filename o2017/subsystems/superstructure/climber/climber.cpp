#include "o2017/subsystems/superstructure/climber/climber.h"
#include <limits>

namespace o2017 {

namespace superstructure {

namespace climber {

Climber::Climber() : position_history_(7) {}

void Climber::SetGoal(const SuperstructureGoalProto& goal) {
  if (goal->should_climb() && current_state() == NOTHING) {
    current_state_ = SPIN_UP;
  } else if (!goal->should_climb()) {
    current_state_ = NOTHING;
  }
}

void Climber::Update(const SuperstructureInputProto& input,
                     const SuperstructureGoalProto& goal,
                     SuperstructureOutputProto* output,
                     SuperstructureStatusProto* status, bool outputs_enabled) {
  SetGoal(goal);

  double voltage = 0.0;
  position_history_.Update(input->climber_position());
  // By finding change divided by time, calculate average rate of change over
  // the range of recorded history
  double current_vel =
      (position_history_.GoBack(0) -
       position_history_.GoBack(position_history_.num_samples() - 1)) /
      (0.005 * position_history_.num_samples());

  if (outputs_enabled) {
    switch (current_state_) {
      case NOTHING:
        voltage = 0.0;
        break;
      case SPIN_UP:
        voltage = kClimbingVoltage;
        if (current_vel > kSpinUpVelocity) {
          current_state_ = APPROACHING;
        }
        break;
      case APPROACHING:
        voltage = kClimbingVoltage;
        if (current_vel < kStartClimbingVelocity) {
          current_state_ = CLIMBING;
        }
        break;
      case CLIMBING:
        voltage = kClimbingVoltage;
        if (current_vel < kFinalVelocity) {
          current_state_ = REACHED_TOP;
        }
        break;
      case REACHED_TOP:
        voltage = kTopVoltage;
        break;
    }
  }

  (*output)->set_climber_voltage(voltage);
  (*status)->set_climber_observed_velocity(current_vel);
  (*status)->set_climber_state(current_state_);
}

ClimberState Climber::current_state() const { return current_state_; }

}  // namespace climber

}  // namespace superstructure

}  // namespace o2017
