#include <limits>
#include "c2017/subsystems/superstructure/climber/climber.h"

namespace c2017 {

namespace climber {

constexpr double kClimbingVoltage = -12.0;
constexpr double kTopVoltage = -2.0;

constexpr double kSpinUpVelocity = 0.42;
constexpr double kStartClimbingVelocity = 0.35;
constexpr double kFinalVelocity = 0.15;

Climber::Climber()
    : position_history_(7), status_queue_(QueueManager::GetInstance()->climber_status_queue()) {}

void Climber::SetGoal(const ClimberGoalProto& goal) {
  if (goal->climbing() && current_state_ == NOTHING) {
    current_state_ = SPIN_UP;
  } else if (!goal->climbing()) {
    current_state_ = NOTHING;
  }
}

ClimberOutputProto Climber::Update(const ClimberInputProto& input, bool outputs_enabled) {
  double voltage_ = 0.0;
  position_history_.Update(input->position());
  // Get the average rate of change over the range of recorded history by finding change divided by time
  double current_vel =
      (position_history_.GoBack(0) - position_history_.GoBack(position_history_.num_samples() - 1)) /
      (0.005 * position_history_.num_samples());

  ClimberStatusProto status;
  ClimberOutputProto output;
  if (outputs_enabled) {
    switch (current_state_) {
      case NOTHING:
        voltage_ = 0;
        break;
      case SPIN_UP:
        voltage_ = kClimbingVoltage;
        if (current_vel > kSpinUpVelocity) {
          current_state_ = APPROACHING;
        }
        break;
      case APPROACHING:
        voltage_ = kClimbingVoltage;
        if (current_vel < kStartClimbingVelocity) {
          current_state_ = CLIMBING;
        }
        break;
      case CLIMBING:
        voltage_ = kClimbingVoltage;
        if (current_vel < kFinalVelocity) {
          current_state_ = AT_TOP;
        }
        break;
      case AT_TOP:
        voltage_ = kTopVoltage;
        break;
    }
  }

  output->set_voltage(voltage_);
  status->set_observed_velocity(current_vel);
  status->set_climber_state(current_state_);

  status_queue_->WriteMessage(status);

  return output;
}

State Climber::current_state() const { return current_state_; }

}  // namespace climber

}  // namespace c2017
