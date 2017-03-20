#include <limits>
#include "c2017/subsystems/superstructure/climber/climber.h"

namespace c2017 {

namespace climber {

constexpr uint32_t kClimbingVoltage = 9;
constexpr uint32_t kSlowingVoltage = 6;

constexpr double kClimbingVelocity = 10;
constexpr double kSlowingVelocity = 6;
constexpr double kFinalVelocity = 0.5;

Climber::Climber()
    : last_position_(0),
      status_queue_(QueueManager::GetInstance().climber_status_queue()) {}

void Climber::SetGoal(const ClimberGoalProto& goal) {
  if (goal->climbing() && current_state_ == NOTHING) {
    current_state_ = APPROACHING;
  } else if (!goal->climbing()) {
    current_state_ = NOTHING;
  }
}

ClimberOutputProto Climber::Update(const ClimberInputProto& input, bool outputs_enabled) {
  double voltage_ = 0.0;
  double current_vel = (input->position() - last_position_) / 0.005;

  ClimberStatusProto status;
  ClimberOutputProto output;
  if (outputs_enabled) {
    switch (current_state_) {
      case NOTHING:
        voltage_ = 0;
        break;
      case APPROACHING:
        voltage_ = kClimbingVoltage;
        if (current_vel > kClimbingVelocity) {
          current_state_ = CLIMBING;
        }
        break;
      case CLIMBING:
        voltage_ = kClimbingVoltage;
        if (current_vel < kSlowingVelocity) {
          current_state_ = SLOWING;
        }
        break;
      case SLOWING:
        voltage_ = kSlowingVoltage;
        if (current_vel < kFinalVelocity) {
          current_state_ = AT_TOP;
        }
        break;
      case AT_TOP:
        voltage_ = 0;
        break;
    }
  }

  last_position_ = input->position();
  output->set_voltage(voltage_);
  status->set_observed_velocity(current_vel);
  status->set_climber_state(current_state_);

  status_queue_.WriteMessage(status);

  return output;
}

State Climber::current_state() const { return current_state_; }

}  // namespace climber

}  // namespace c2017
