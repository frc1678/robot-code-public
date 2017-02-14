#include "c2017/subsystems/superstructure/ground_gear_intake/ground_gear_intake.h"
#include "muan/wpilib/queue_types.h"

namespace c2017 {

namespace ground_gear_intake {

GroundGearIntakeOutputProto GroundGearIntake::Update(GroundGearIntakeInputProto input, bool outputs_enabled) {
  double voltage = 0;

  if (outputs_enabled) {
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
  }
  GroundGearIntakeOutputProto output;
  GroundGearIntakeStatusProto ground_gear_status;
  output->set_roller_voltage(voltage);
  output->set_intake_down(intake_down_);
  ground_gear_status->set_current_spiked(has_current_spiked_);
  ground_gear_status->set_down(intake_down_);
  ground_gear_status->set_running(fabs(voltage) <= 1e-3);
  QueueManager::GetInstance().ground_gear_status_queue().WriteMessage(ground_gear_status);
  return output;  // Sends voltage and solenoid output
}

void GroundGearIntake::SetGoal(GroundGearIntakeGoalProto goal) { goal_state_ = goal->goal(); }

}  // namespace ground_gear_intake

}  // namespace c2017
