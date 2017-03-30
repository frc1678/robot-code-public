#include "c2017/subsystems/superstructure/ground_gear_intake/ground_gear_intake.h"
#include "muan/wpilib/queue_types.h"

namespace c2017 {

namespace ground_gear_intake {

GroundGearIntakeOutputProto GroundGearIntake::Update(GroundGearIntakeInputProto input, bool outputs_enabled) {
  double voltage = 0;
  bool intake_down = false;
  bool current_spiked = false;

  if (outputs_enabled) {
    switch (current_state_) {
      case IDLE:
        voltage = 0.0;
        intake_down = false;
        break;
      case INTAKING:
        voltage = kIntakeVoltage;
        intake_down = true;
        if (input->current() > kCurrentThreshold) {
          current_spiked = true;
          current_state_ = PICKING_UP;
          pickup_timer_ = kPickupTicks;
          voltage = kPickupVoltage;
        }
        break;
      case PICKING_UP:
        voltage = kPickupVoltage;
        intake_down = false;
        if (--pickup_timer_ < 0) {
          current_state_ = CARRYING;
        }
        break;
      case CARRYING:
        voltage = kCarryVoltage;
        intake_down = false;
        break;
      case SCORING:
        voltage = kScoreVoltage;
        intake_down = false;
        break;
      case OUTTAKING:
        voltage = kOuttakeVoltage;
        intake_down = false;
    }
  }

  GroundGearIntakeOutputProto output;
  GroundGearIntakeStatusProto ground_gear_status;
  output->set_roller_voltage(voltage);
  output->set_intake_down(intake_down);
  ground_gear_status->set_current_spiked(current_spiked);
  ground_gear_status->set_current_state(current_state_);
  ground_gear_status->set_running(fabs(voltage) <= 1e-3);
  QueueManager::GetInstance().ground_gear_status_queue().WriteMessage(ground_gear_status);
  return output;  // Sends voltage and solenoid output
}

void GroundGearIntake::SetGoal(GroundGearIntakeGoalProto goal) {
  switch (goal->goal()) {
    case DROP:
      current_state_ = INTAKING;
      break;
    case RISE:
      if (current_state_ == INTAKING) {
        current_state_ = IDLE;
      }
      break;
    case SCORE:
      if (current_state_ == CARRYING || current_state_ == IDLE) {
        current_state_ = SCORING;
      }
      break;
    case NONE:
      if (current_state_ == SCORING) {
        current_state_ = IDLE;
      }
      break;
    case OUTTAKE:
      current_state_ = OUTTAKING;
      break;
  }
}

State GroundGearIntake::current_state() const { return current_state_; }

}  // namespace ground_gear_intake

}  // namespace c2017
