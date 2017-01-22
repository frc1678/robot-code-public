#include "ground_gear_intake.h"

namespace c2017 {

namespace ground_gear_intake {

GroundGearIntakeOutputProto GroundGearIntake::Update(GroundGearIntakeInputProto input) {
  double voltage = 0;

  switch (goal_state_) {
    case CARRY:
      voltage = 0;
      intake_down_ = false;
      has_current_spiked_ = false;
      break;

    case PICKUP:
      voltage = 12;
      intake_down_ = true;
      if (input->current() > 120) {  // the intake stalls when its current spikes
        has_current_spiked_ = true;
      }
      if (has_current_spiked_) {
        voltage = 0;  // stop moving when the motor stalls
        intake_down_ = false;
      }
      break;

    case SCORE:
      voltage = -12;  // outtake
      intake_down_ = false;
      has_current_spiked_ = false;
      break;
  }  
  
  GroundGearIntakeOutputProto output;
  output->set_roller_voltage(voltage);
  output->set_intake_down(intake_down_);
  return output; // sends voltage and solenoid output

}

void GroundGearIntake::SetGoal(GroundGearIntakeGoalProto goal) { goal_state_ = goal->goal(); }

}  // ground_gear_intake

}  // c2017
