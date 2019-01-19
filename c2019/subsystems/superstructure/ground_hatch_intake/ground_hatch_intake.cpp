#include "c2019/subsystems/superstructure/ground_hatch_intake/ground_hatch_intake.h"

namespace c2019 {
namespace ground_hatch_intake {

void GroundHatchIntake::Update(bool outputs_enabled,
                               const GroundHatchIntakeInputProto& input,
                               GroundHatchIntakeOutputProto* output,
                               GroundHatchIntakeStatusProto* status) {
  double voltage = 0;
  bool snap_down = false;

  if (outputs_enabled) {
    switch (current_state_) {
      case IDLE:
        voltage = 0;
        snap_down = false;
      case INTAKING:
        voltage = kIntakeVoltage;
        snap_down = true;
      case HOLDING:
        voltage = kHoldVoltage;
        snap_down = false;
      case OUTTAKING:
        voltage = kOuttakeVoltage;
        snap_down = true;
    }
  }

  (*output)->set_roller_voltage(voltage);
  (*output)->set_snap_down(snap_down);
  (*status)->set_state(current_state_);
  (*status)->set_has_hatch(input->current() > kCurrentThreshold);
}

void GroundHatchIntake::SetGoal(GroundHatchIntakeGoalProto goal) {
  switch (goal->goal()) {
    case NONE:
      current_state_ = IDLE;
      break;
    case INTAKE:
      if (current_state_ != HOLDING) {
        current_state_ = INTAKING;
      }
      break;
    case HOLD:
      current_state_ = HOLDING;
      break;
    case OUTTAKE:
      current_state_ = OUTTAKING;
      break;
  }
}

}  // namespace ground_hatch_intake
}  // namespace c2019
